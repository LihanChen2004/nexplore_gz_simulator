#ifndef NEXPLORE_GZ_SIMULATOR_H_
#define NEXPLORE_GZ_SIMULATOR_H_

#include <gazebo_msgs/ModelState.h>
#include <gazebo_msgs/SpawnModel.h>
#include <nexplore/GetDatasetConfig.h>
#include <nexplore_gz_simulator/GetPointCloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/package.h>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include <condition_variable>

namespace nexplore_gz_simulator
{
class NexploreGZSimulator
{
public:
  NexploreGZSimulator();
  void scanHandler(const sensor_msgs::PointCloud2::ConstPtr & scanIn);
  bool getPointCloudCallback(
    nexplore_gz_simulator::GetPointCloud::Request & req,
    nexplore_gz_simulator::GetPointCloud::Response & res);
  void initGazebo(const nexplore::GetDatasetConfig::Response & res);
  void modelUpdate(const ros::TimerEvent & event);
  void publishTransform(
    const std::string & parent, const std::string & child, double x, double y, double z,
    const tf2::Quaternion & quat);
  void replaceAll(std::string & str, const std::string & from, const std::string & to);

private:
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;
  ros::Subscriber pcd_sub_;
  ros::Publisher model_state_pub_;
  ros::ServiceServer get_pcd_server_;
  ros::ServiceClient get_dataset_client_;
  ros::Timer timer_;
  tf::TransformBroadcaster tf_broadcaster_;
  tf2_ros::Buffer tfBuffer_;
  tf2_ros::TransformListener tfListener_;

  sensor_msgs::PointCloud2::ConstPtr pcd_;
  gazebo_msgs::ModelState lidarState;

  float lidar_x_;
  float lidar_y_;
  float lidar_z_;
  geometry_msgs::Quaternion lidar_quat_;

  std::mutex mtx_;
  std::condition_variable cv_;
  bool wait_for_pcd_ = false;
  bool pcd_updated_ = false;
};

}  // namespace nexplore_gz_simulator

#endif
