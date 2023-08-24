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
- Greenhouse(w/o GNSS): [LINK](https://drive.google.com/drive/folders/11z2fR-tdS2LTLQ7sVV_8QxDRo-XrNlK1?usp=drive_link)
- Outdoor walkways on campus: [LINK](https://drive.google.com/drive/folders/16aOzXaX3CCX9drnYVY8J4Pu-ovgsHn2R?usp=drive_link)
- Nature Trail: [Scene1-4](https://drive.google.com/drive/folders/1gWCEeUI05r3FNMN8ulX8S7LYm3LC-RAb?usp=sharing), [Scene5-8](https://drive.google.com/drive/folders/1QZnQl6IZU9FYfL6_Mjj664fkJw4WEjzd?usp=drive_link), [Scene9-12](https://drive.google.com/drive/folders/1q2m0HaGGOgZMB15ceJB5kXypd5kvtxIG?usp=drive_link), [Scene13-16](https://drive.google.com/drive/folders/183sQTOaI1PWonWXeDKUbcbaRW2QZ34u4?usp=drive_link)
