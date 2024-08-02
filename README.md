# NExplore_GZ_Simulator

This Gazebo Simulator derived from [NExplore](https://github.com/kzj18/NExplore) is mainly used to obtain pointcloud.

## Requirement

- OS: Ubuntu20.04
- ROS: Noetic
- Gazebo 11

## Usage

### Build

```sh
cd ros_ws/src
git clone https://github.com/LihanChen2004/nexplore_gz_simulator.git
```

```sh
cd ..
rosdep install -r --from-paths src --ignore-src --rosdistro $ROS_DISTRO -y
```

```sh
catkin_make
```

### Launch

```sh
roslaunch nexplore_gz_simulator nexplore_gz_simulator.launch
```

## Informations

This package requires [NExplore](https://github.com/kzj18/NExplore) for complete functionality.

1. Create a client [`/get_dataset_config`](https://github.com/kzj18/NExplore/blob/master/srv/GetDatasetConfig.srv) to retrieve the mesh URL and pose for spawning the mesh in Gazebo.

2. Create a server [`/get_pointcloud`](./nexplore_gz_simulator/srv/GetPointCloud.srv) to spawn the LiDAR pose and obtain the point cloud.

    The response pointcloud is based on the frame `world_nexplore`, which uses the OpenCV coordinate system.
