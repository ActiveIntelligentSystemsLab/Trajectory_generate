# About

A package that uses rtabmap to automatically draw the start and end points in an image and create a dataset.  
You will need to install rtabmap_ros to use this package.

# Usage

## created map
---
To create a map using this package, run the following command

```code:bash
$roslaunch rtabmap_create_data rosbag_to_map.launch rosbag:=true image:=/kinect2/qhd/image_color_rect depth:=/kinect2/qhd/image_depth_rect camera_info:=/kinect2/qhd/camera_info odom:=/kinect2_link
```

If data is taken with Realsense, the following launch file can be used to create it.

```code:bash
$roslaunch rtabmap_create_data rosbag_to_map_realsensed435i.launch
```

Once the launch file has been launched, playing the rosbag will automatically create the map.

```code:bash
$rosbag play --clock src/rosbag_data/26_4_filtered.bag
```

Once the map data has been created, it can be displayed by executing the following command.

```terminal:map view
$roslaunch rtabmap_ros rtabmap.launch database_path:="~/.ros/rtabmap.db"
```

Replace database_path with anything you want and run it.
Execute this command to launch RtabMap. Click "Download map" and select "local map optimized".

## create dataset command
---
Check that the map has been created.
Then run the following command.

```terminal:execute create dataset
$rosrun rtabmap_create_data map_to_image
```

You can then make a service call to create the dataset.

```terminal:create data
$rosservice call /simple_demo_node/get_octomap
$rosservice call /simple_demo_node/get_octomap "name:'test'"
```

Argment

- name : create csv name(String)

# Trajectory and Image
The data created by this script is available at the following link.
- Greenhouse: LINK
- Sakaki Park: [Image](https://drive.google.com/drive/folders/1r86VdlAFYoo2JnIbW15G_UQ6H2MAt2yx?usp=sharing), [Trajectory](https://drive.google.com/drive/folders/1KBcuvrlyDnOzq-mb_UB8wOSmx1zKtmCL?usp=sharing)
- Nature Trail: [Image](https://drive.google.com/drive/folders/15rbCDSwCrZfFvDKyYNDqzUDZ8Ka230K1?usp=sharing), [Trajectory](https://drive.google.com/drive/folders/1eDfGuTIOxAVseU5C7nzBpRBjZNfr0CJm?usp=sharing)

# ROS Bag Raw Data
Data included in raw ROS bagfiles:
| Topic Name | Message Type | Description |
| ---- | ---- | ---- |
| /camera/aligned_depth_to_color/camera_info | sensor_msgs/CameraInfo | |
| /camera/aligned_depth_to_color/image_raw | sensor_msgs/Image | |
| /camera/color/camera_info | sensor_msgs/CameraInfo | |
| /camera/color/image_raw | sensor_msgs/Image | |
| /camera/depth/camera_info | sensor_msgs/CameraInfo | |
| /camera/depth/image_rect_raw | sensor_msgs/Image | |
| /camera/extrinsics/depth_to_color | realsense2_camera/Extrinsics | |
| /camera/gyro/imu_info | realsense2_camera/IMUInfo | |
| /camera/imu | sensor_msgs/Imu | |
| /tf_static | tf2_msgs/TFMessage | |
| /ublox/fix | sensor_msgs/NavSatFix | |
| /velodyne_points | sensor_msgs/PointCloud2 | |

## ROS Bag Download
The following are the links for the ROS Bag files.
- Greenhouse(w/o GNSS): [LINK](https://drive.google.com/drive/folders/1SkQqjOVCEIfVtKqkaTqsMY7RXUrXQG5V?usp=sharing)
- Outdoor walkways on campus: [LINK](https://drive.google.com/drive/folders/1gDawJ0bAbt2GLxvHze2Nx5NkuKjG-wsh?usp=sharing)
- Nature Trail: [Scene1-4](https://drive.google.com/drive/folders/1b4yKDzp4CuS5ZxjXa8uKEnFmI9BVmKmR?usp=sharing), [Scene5-8](https://drive.google.com/drive/folders/1OmlrXeYCnDlVh8Jz5b9GL6muJiFTkMAE?usp=sharing), [Scene9-12](https://drive.google.com/drive/folders/1v114hAQN5aPZ7Q1IOLdVD0n6DpXMH1FX?usp=sharing), [Scene13-16](https://drive.google.com/drive/folders/1pIk_HUDlL_WFV8tEvr7qgQVr90c1gAVQ?usp=sharing)
