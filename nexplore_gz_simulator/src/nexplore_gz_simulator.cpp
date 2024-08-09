#include "nexplore_gz_simulator/nexplore_gz_simulator.h"

#include <pcl_ros/transforms.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace nexplore_gz_simulator
{
NexploreGZSimulator::NexploreGZSimulator()
: nh_private_("~"), tfListener_(tfBuffer_), lidar_x_(0.0), lidar_y_(0.0), lidar_z_(0.0)
{
  pcd_sub_ = nh_.subscribe<sensor_msgs::PointCloud2>(
    "/velodyne_points", 1, &NexploreGZSimulator::scanHandler, this);
  model_state_pub_ = nh_.advertise<gazebo_msgs::ModelState>("/gazebo/set_model_state", 5);

  get_pcd_server_ =
    nh_.advertiseService("/get_pointcloud", &NexploreGZSimulator::getPointCloudCallback, this);
  get_dataset_client_ = nh_.serviceClient<nexplore::GetDatasetConfig>("/get_dataset_config");

  timer_ = nh_.createTimer(ros::Duration(0.01), &NexploreGZSimulator::modelUpdate, this);

  nh_private_.getParam("lidar_offset_x", lidar_offset_x_);
  nh_private_.getParam("lidar_offset_y", lidar_offset_y_);
  nh_private_.getParam("lidar_offset_z", lidar_offset_z_);
  nh_private_.getParam("lidar_offset_roll", lidar_offset_roll_);
  nh_private_.getParam("lidar_offset_pitch", lidar_offset_pitch_);
  nh_private_.getParam("lidar_offset_yaw", lidar_offset_yaw_);
  ROS_INFO("Set lidar_offset_x to: %f", lidar_offset_x_);
  ROS_INFO("Set lidar_offset_y to: %f", lidar_offset_y_);
  ROS_INFO("Set lidar_offset_z to: %f", lidar_offset_z_);
  ROS_INFO("Set lidar_offset_roll to: %f", lidar_offset_roll_);
  ROS_INFO("Set lidar_offset_pitch to: %f", lidar_offset_pitch_);
  ROS_INFO("Set lidar_offset_yaw to: %f", lidar_offset_yaw_);

  nexplore::GetDatasetConfig srv;
  ros::service::waitForService("/get_dataset_config");
  if (get_dataset_client_.call(srv)) {
    initGazebo(srv.response);
  } else {
    ROS_ERROR("Failed to call service get_dataset_config");
  }
}

void NexploreGZSimulator::modelUpdate(const ros::TimerEvent &)
{
  tf2::Quaternion temp_quat;
  tf2::convert(lidar_quat_, temp_quat);
  publishTransform("world_gazebo", "velodyne", lidar_x_, lidar_y_, lidar_z_, temp_quat);

  tf2::Quaternion worldNexploreQuat;
  worldNexploreQuat.setRPY(M_PI / 2, 0, 0);
  publishTransform("world_gazebo", "world_nexplore", 0, 0, 0, worldNexploreQuat);
}

void NexploreGZSimulator::publishTransform(
  const std::string & parent_frame, const std::string & child_frame, double x, double y, double z,
  const tf2::Quaternion & quat)
{
  tf2::Quaternion temp_quat = quat;
  // Check if the quaternion is valid
  if (quat.length2() < 0.999 || quat.length2() > 1.001) {
    ROS_WARN("Invalid quaternion, set to default quaternion");
    temp_quat = tf2::Quaternion(0, 0, 0, 1);
  }
  geometry_msgs::TransformStamped transformStamped;
  transformStamped.header.stamp = ros::Time::now();
  transformStamped.header.frame_id = parent_frame;
  transformStamped.child_frame_id = child_frame;
  transformStamped.transform.translation.x = x;
  transformStamped.transform.translation.y = y;
  transformStamped.transform.translation.z = z;
  transformStamped.transform.rotation = tf2::toMsg(temp_quat);
  tf_broadcaster_.sendTransform(transformStamped);
}

void NexploreGZSimulator::scanHandler(const sensor_msgs::PointCloud2::ConstPtr & scanIn)
{
  std::lock_guard<std::mutex> lock(mtx_);
  pcd_ = scanIn;
  if (wait_for_pcd_ & (scanIn->header.stamp > request_pcd_time_)) {
    pcd_updated_ = true;
    cv_.notify_all();
    ROS_DEBUG("Point cloud updated and notified");
  }
}

bool NexploreGZSimulator::getPointCloudCallback(
  nexplore_gz_simulator::GetPointCloud::Request & req,
  nexplore_gz_simulator::GetPointCloud::Response & res)
{
  std::unique_lock<std::mutex> lock(mtx_);

  request_pcd_time_ = req.pose.header.stamp;

  // Get transform from world_gazebo to world_nexplore
  geometry_msgs::TransformStamped transformStamped;
  try {
    transformStamped =
      tfBuffer_.lookupTransform("world_gazebo", "world_nexplore", ros::Time(0), ros::Duration(3.0));
  } catch (const tf2::TransformException & ex) {
    ROS_WARN("Transform lookup failed: %s", ex.what());
    return false;
  }

  // Transform the req.pose from world_nexplore to world_gazebo
  geometry_msgs::PoseStamped poseInNexplore, poseInGazebo;
  poseInNexplore.header.frame_id = "lidar_nexplore";
  poseInNexplore.pose = req.pose.pose;
  tf2::doTransform(poseInNexplore, poseInGazebo, transformStamped);

  tf2::Quaternion temp_quat;
  tf2::fromMsg(poseInGazebo.pose.orientation, temp_quat);

  // Rotate lidar frame to match ros axis
  tf2::Quaternion q1, q2, rotatedQuat;
  q1.setRPY(-M_PI / 2, 0, 0);
  q2.setRPY(0, 0, M_PI / 2);
  rotatedQuat = temp_quat * q1 * q2;

  // Set LiDAR offset pose, based on camera pose from nexplore
  poseInGazebo.pose.position.x += lidar_offset_x_;
  poseInGazebo.pose.position.y += lidar_offset_y_;
  poseInGazebo.pose.position.z += lidar_offset_z_;
  tf2::Quaternion offsetQuat;
  offsetQuat.setRPY(lidar_offset_roll_, lidar_offset_pitch_, lidar_offset_yaw_);
  rotatedQuat *= offsetQuat;

  // Update LiDAR model in gazebo
  lidar_x_ = poseInGazebo.pose.position.x;
  lidar_y_ = poseInGazebo.pose.position.y;
  lidar_z_ = poseInGazebo.pose.position.z;
  lidar_quat_ = tf2::toMsg(rotatedQuat);

  lidarState.model_name = "lidar";
  lidarState.pose.orientation = lidar_quat_;
  lidarState.pose.position.x = lidar_x_;
  lidarState.pose.position.y = lidar_y_;
  lidarState.pose.position.z = lidar_z_;
  model_state_pub_.publish(lidarState);

  // Wait for point cloud to update
  wait_for_pcd_ = true;
  ROS_DEBUG("Waiting for point cloud to update");
  cv_.wait(lock, [this] { return pcd_updated_; });

  // Transform point cloud to world_nexplore frame
  sensor_msgs::PointCloud2 transformedPcd;
  try {
    transformStamped = tfBuffer_.lookupTransform(
      "world_nexplore", pcd_->header.frame_id, pcd_->header.stamp, ros::Duration(3.0));
    pcl_ros::transformPointCloud("world_nexplore", *pcd_, transformedPcd, tfBuffer_);
  } catch (const tf2::TransformException & ex) {
    ROS_WARN("Failed to transform point cloud: %s", ex.what());
    return false;
  }

  res.pointcloud = transformedPcd;
  ROS_DEBUG("Send point cloud data");

  pcd_updated_ = false;
  wait_for_pcd_ = false;
  return true;
}

void NexploreGZSimulator::initGazebo(const nexplore::GetDatasetConfig::Response & res)
{
  ros::service::waitForService("gazebo/spawn_sdf_model");
  ros::NodeHandle n;
  auto client = n.serviceClient<gazebo_msgs::SpawnModel>("gazebo/spawn_sdf_model");
  gazebo_msgs::SpawnModel spawnModel;

  std::string model_sdf_path =
    ros::package::getPath("nexplore_gz_simulator") + "/mesh/templates/model.sdf";
  std::ifstream ifs(model_sdf_path);
  std::string model_xml((std::istreambuf_iterator<char>(ifs)), std::istreambuf_iterator<char>());

  std::string mesh_url = res.scene_mesh_url;
  replaceAll(mesh_url, ".glb", ".dae");
  replaceAll(model_xml, "[mesh_file_uri]", mesh_url);

  geometry_msgs::TransformStamped transformStamped;
  try {
    transformStamped =
      tfBuffer_.lookupTransform("world_gazebo", "world_nexplore", ros::Time(0), ros::Duration(3.0));
  } catch (tf2::TransformException & ex) {
    ROS_WARN("%s", ex.what());
  }

  geometry_msgs::QuaternionStamped quat_in_nexplore, quat_in_gazebo;
  quat_in_nexplore.header.frame_id = "world_nexplore";
  quat_in_nexplore.quaternion = res.scene_mesh_transform.orientation;
  tf2::doTransform(quat_in_nexplore, quat_in_gazebo, transformStamped);

  spawnModel.request.model_name = "gazebo_world";
  spawnModel.request.initial_pose.orientation = quat_in_gazebo.quaternion;
  spawnModel.request.model_xml = std::move(model_xml);

  if (client.call(spawnModel)) {
    ROS_INFO("Model path: %s", mesh_url.c_str());
    ROS_INFO("Successfully spawned the model");
  } else {
    ROS_ERROR("Failed to spawn world model");
  }
}

void NexploreGZSimulator::replaceAll(
  std::string & str, const std::string & from, const std::string & to)
{
  size_t pos = 0;
  while ((pos = str.find(from, pos)) != std::string::npos) {
    str.replace(pos, from.length(), to);
    pos += to.length();
  }
}

}  // namespace nexplore_gz_simulator

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "NexploreGZSimulator");
  nexplore_gz_simulator::NexploreGZSimulator nexploreGZSimulator;
  ros::MultiThreadedSpinner spinner(4);
  spinner.spin();
  return 0;
}
