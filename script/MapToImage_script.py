#!/usr/bin/python
#-*- coding: utf-8 -*-

# ros package
from email.mime import base
import rospy
import rosparam
import rtabmap_ros
import tf
import tf2_ros
import math

# ros package msgs file
from rtabmap_ros.srv import GetMap
from rtabmap_ros.msg import NodeData
from geometry_msgs.msg import TransformStamped, Transform, Point, Vector3, Quaternion, Pose
from sensor_msgs.msg import Image, PointCloud2, CameraInfo
from rtabmap_create_data.srv import CreateData
from cv_bridge import CvBridge, CvBridgeError
from visualization_msgs.msg import MarkerArray, Marker

# python package
import numpy as np

# image library
import cv2
import csv
from PIL import Image, ImageDraw

# SAVE_PATH = "/media/aisl/HD-PGF-A1/Share_data/M2_Data_Share/Sakaki_bag/20221221/with_odom_gps/trial2/"
SAVE_PATH = "/media/aisl/HD-PGF-A1/Share_data/M2_Data_Share/Greenhouse/"
IMAGE_NAME = "greenhouse_scene02"
PATH_WIDTH = 0.3

trajectoryMarkerPub = rospy.Publisher('trajectory', MarkerArray, queue_size=10)

def generateData(serviceCall):
    """_summary_

    Args:
        serviceCall (object): tuple of csvName and datasetName

    Returns:
        boolean : servicecall successed?
    """
    # start service call
    rospy.loginfo("[callback Get image and trajectory] start!")

    # dataset csv name
    dataset_csv_name = serviceCall.csvName

    # define tf_broadcaster and tf_listener
    tf_broadcast = tf2_ros.TransformBroadcaster()
    tf_Buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_Buffer)

    # trigonometric = calcCosSin(rosparam.get_param("/robot/angle_x"))
    # trigonometric_ = calcCosSin(rosparam.get_param("/robot/angle_y"))
    # service call rtabmap
    rospy.wait_for_service("/rtabmap/get_map_data")

    loop_num = rosparam.get_param("/generate/loop_num")
    point_num = rosparam.get_param("/generate/point_num")

    path_width = rosparam.get_param("/path/width")

    try:
        service_proxy = rospy.ServiceProxy("/rtabmap/get_map_data", GetMap)

        # service call parameter: global_, optimized, graphOnly
        service_rep = service_proxy(True, True, False)
        if(service_rep):
            mapData = service_rep.data
            rospy.loginfo("[GetMap Service Call Result]Get MapData!")
        else:
            print("Failed")
            return False
    except rospy.ServiceException as e:
        print("Service Call Failed: %s" % e)
        return False

    # ======= Start collback function (calcurated point data) =======
    rospy.loginfo("[MapData Config total size] Pose Id : %d, Poses : %d, Nodes : %d", 
                    len(mapData.graph.posesId), len(mapData.graph.poses), len(mapData.nodes))

    # Get camera parameter
    frame_data = mapData.nodes[0]
    width, height = frame_data.width[0], frame_data.height[0]
    cx, cy, fx, fy = frame_data.cx[0], frame_data.cy[0], frame_data.fx[0], frame_data.fy[0]
    rospy.loginfo("[Camera Parameter : image size] width : %d, height, %d",
                    width, height)
    rospy.loginfo("[Camera Parameter : Internal Parameters] Cx : %d, Cy : %d, Fx : %d, Fy : %d",
                    cx, cy, fx, fy)
    # set camera parameter
    K = [fx, 0, cx, 0, fy, cy, 0, 0, 1]
    base_to_camera = frame_data.localTransform[0]
    recordTransform(tf_broadcast, mapData.graph.mapToOdom.translation, mapData.graph.mapToOdom.rotation, "map", "map_to_odom")

    flag_update = False
    # translate position is low?(use rosparameter: /robot/height and /robot/width)
    if base_to_camera.translation.z < 0.1:
        base_to_camera.translation.z = rosparam.get_param("/robot/height")
        base_to_camera.translation.y = rosparam.get_param("/robot/width")
        base_to_camera.translation.x = rosparam.get_param("/robot/depth")
        flag_update = True

        rospy.loginfo("[Warning!] update NodeData parameter(Camera position.z)! Please check mapData.")

    rospy.loginfo("height: %f, width: %f, angle_x: %d, angle_y: %d",
                base_to_camera.translation.z,base_to_camera.translation.y,
                rosparam.get_param("/robot/angle_x"), rosparam.get_param("/robot/angle_y"))

    # node pose list generate
    node_datalist = [(mapData.graph.poses[index].position.x, mapData.graph.poses[index].position.y, mapData.graph.poses[index].position.z, \
                        mapData.graph.poses[index].orientation.x, mapData.graph.poses[index].orientation.y, mapData.graph.poses[index].orientation.z, mapData.graph.poses[index].orientation.w)
                        for index in range(len(mapData.graph.poses))]
    rospy.loginfo("[Info] Publish Trajectory point")
    # visualizePoint(node_datalist, base_to_camera.translation)

    if loop_num == 0:
        loop_range = len(mapData.nodes) - 1
    else:
        loop_range = loop_num

    if not flag_update:
        estimate_frame = "trajectory_point_"
        change_frame = "camera_point_"
    else:
        estimate_frame = "camera_point_"
        change_frame = "trajectory_point_"

    # regist transform(tf2)
    for index in range(len(node_datalist)):
        recordTransform(tf_broadcast, (node_datalist[index][0], node_datalist[index][1], node_datalist[index][2]),
                        (node_datalist[index][3], node_datalist[index][4], node_datalist[index][5], node_datalist[index][6]),
                        "map_to_odom", estimate_frame + str(index))
        recordTransform(tf_broadcast, base_to_camera.translation, base_to_camera.rotation,
                        estimate_frame + str(index), change_frame + str(index))
        recordTransform(tf_broadcast, (0, -path_width, 0), (0, 0, 0, 1),
                        estimate_frame + str(index), estimate_frame + str(index) + "_right")
        recordTransform(tf_broadcast, (0, path_width, 0), (0, 0, 0, 1),
                        estimate_frame + str(index), estimate_frame + str(index) + "_left")
    coordinate_list = []
    for node_num_index in range(loop_range):
        node_data = mapData.nodes[node_num_index]
        image_data = node_data.image
        # image decoding
        try:
            np_arr = np.fromstring(image_data, np.uint8)
            convert_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        except CvBridgeError as e:
            rospy.loginfo("[Error!] image convert error!")

        # image data load
        image_pil = cv2.cvtColor(convert_image, cv2.COLOR_BGR2RGB)
        image_pil = Image.fromarray(image_pil)
        canvas = ImageDraw.Draw(image_pil)
        if rosparam.get_param("/generate/generate_origin"):
            image_pil.save(SAVE_PATH + IMAGE_NAME + "_" + str(node_num_index).zfill(4) + ".png")

        # define loop number
        max_index_range = point_num if node_num_index + point_num < len(mapData.nodes) else len(mapData.nodes) - node_num_index

        left_line, right_line = [], []
        trans_coord_list, trans_coord = [], []
        for point_loop in range(1, max_index_range):
            try:
                trans = tf_Buffer.lookup_transform("camera_point_" + str(node_num_index), 'trajectory_point_' + str(node_num_index + point_loop), rospy.Time(0))
            except(tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException,) as e:
                rospy.loginfo("[ERROR!!] don't calc!")
                return False

            trans_coordinate = trans.transform.translation
            if not flag_update:
                image_point_x, image_point_y, image_point_z = trans_coordinate.x, trans_coordinate.y, trans_coordinate.z
            else:
                image_point_x, image_point_y, image_point_z = -trans_coordinate.y, -trans_coordinate.z, trans_coordinate.x

            # Restrictions on data to be drawn
            if image_point_z < 0.0:
                continue

            # after update
            coordinates = imageCoordTransform([image_point_x, image_point_y, image_point_z], K)
            trans_coord.append([int(coordinates[0]), int(coordinates[1])])

            coord_x, coord_y = int(coordinates[0]), int(coordinates[1])
            trans_coord_list.append((coord_x, coord_y))

            # Coordinate calculation for annotation
            try:
                trans_left = tf_Buffer.lookup_transform("camera_point_" + str(node_num_index), 'trajectory_point_' + str(node_num_index + point_loop) + "_left", rospy.Time(0))
                trans_right = tf_Buffer.lookup_transform("camera_point_" + str(node_num_index), 'trajectory_point_' + str(node_num_index + point_loop) + "_right", rospy.Time(0))
            except(tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException,) as e:
                rospy.loginfo("[ERROR!!] don't calc!")
                return False

            image_point_right = trans_right.transform.translation
            image_point_left = trans_left.transform.translation

            if point_loop > 5 and (coordinates[0] < 0 or coordinates[0] > width or coordinates[1] < 0 or coordinates[1] > height):
                continue
            # robot pattern
            right_line.append(imageCoordTransform([image_point_right.x, image_point_right.y, image_point_right.z], K))
            left_line.append(imageCoordTransform([image_point_left.x, image_point_left.y, image_point_left.z], K))

        coordinate_list.append(trans_coord)

        if rosparam.get_param("/generate/dataset_polygon"):
            # generate polygon
            right_line.reverse()
            # drawing polygon(annotation area)
            polygon_data = left_line + right_line
            polygon_data.append(left_line[0])
            canvas.polygon(polygon_data, fill=(255, 255, 0))

        if rosparam.get_param("/generate/dataset_point"):
            for coord in trans_coord_list:
                canvas.ellipse(((coord[0] - 5, coord[1] - 5), (coord[0] + 5, coord[1] + 5)), fill=(255, 0, 0))

        image_pil.save(SAVE_PATH + "point_data/" + IMAGE_NAME + "_" + str(node_num_index).zfill(4) + ".png")

    # write csv
    with open(SAVE_PATH + serviceCall.csvName + ".csv", "wb") as f:
        csv_data, header_data = [], []
        header_data.append("image_path")
        for index_num in range(point_num - 1):
            header_data.append("path_num_" + str(index_num + 1))
        csv_data.append(header_data)
        for index, data in enumerate(coordinate_list):
            temp_data = []
            path_name = "/Nature_env_dataset/" + serviceCall.datasetName + "/" + IMAGE_NAME + "_" + str(index).zfill(4) + ".png"
            temp_data = [coordinate for coordinate in data]
            temp_data.insert(0, path_name)
            csv_data.append(temp_data)
        writer = csv.writer(f)
        writer.writerows(csv_data)

    return True

def calcCosSin(degree):
    thete = math.radians(degree/2)
    cos = math.cos(thete)
    sin = math.sin(thete)
    return (cos, sin)

def appendList(coordinates, width, height):
    if coordinates[0] < 0:
        coordinate_x = 0
    elif coordinates[0] > width:
        coordinate_x = width
    else:
        coordinate_x = coordinates[0]

    if coordinates[1] < 0:
        coordinate_y = 0
    elif coordinates[1] > height:
        coordinate_y = height
    else:
        coordinate_y = coordinates[1]

    return (coordinate_x, coordinate_y)

def recordTransform(broadcaster, position_, rotation_, frame_id, child_frame):
    # define msg
    transform = TransformStamped()
    transform.header.frame_id = frame_id
    transform.child_frame_id = child_frame

    # argument check
    if isinstance(position_, list) or isinstance(position_, tuple):
        position = Vector3()
        rotation = Quaternion()
        position.x, position.y, position.z = position_[0], position_[1], position_[2]
        rotation.x, rotation.y, rotation.z, rotation.w = rotation_[0], rotation_[1], rotation_[2], rotation_[3]
    else:
        position, rotation = position_, rotation_

    transform.header.stamp = rospy.Time.now()
    transform.transform.translation = position
    transform.transform.rotation = rotation

    broadcaster.sendTransform(transform)

def imageCoordTransform(point, parameter_K):
    coord_x = (point[0] * parameter_K[0]) / point[2] + parameter_K[2]
    coord_y = (point[1] * parameter_K[4]) / point[2] + parameter_K[5]
    return (coord_x, coord_y)

def visualizePoint(point_list, transform_point_list):
    publish_point_data = MarkerArray()
    # Drawing Trajectory
    path_point_msg = Marker()
    # Transform Trajectory(/camera_link -> /base_link)
    trans_path_point_msg = Marker()
    # config marker
    path_point_msg.header.frame_id = trans_path_point_msg.header.frame_id = "/map"
    path_point_msg.ns = "trajectory"
    trans_path_point_msg.ns = "trasnTrajectory"
    path_point_msg.type = trans_path_point_msg.type = path_point_msg.SPHERE_LIST
    path_point_msg.action = trans_path_point_msg.action = path_point_msg.ADD

    # Add Point List
    for point_data_ in point_list:
        point_data = Point()
        trans_data = Point()
        point_data.x, point_data.y, point_data.z = point_data_[0], point_data_[1], point_data_[2]
        trans_data.x, trans_data.y, trans_data.z = point_data_[0] + transform_point_list.x, point_data_[1] + transform_point_list.y , point_data_[2] + transform_point_list.z
        path_point_msg.points.append(point_data)
        trans_path_point_msg.points.append(trans_data)
    path_point_msg.scale.x = path_point_msg.scale.y = path_point_msg.scale.z = 0.05
    trans_path_point_msg.scale.x = trans_path_point_msg.scale.y = trans_path_point_msg.scale.z = 0.05
    path_point_msg.color.r = path_point_msg.color.a = 1.0
    trans_path_point_msg.color.b = trans_path_point_msg.color.a = 1.0
    path_point_msg.color.g = path_point_msg.color.b = 0.0
    trans_path_point_msg.color.r = trans_path_point_msg.color.g = 0.0

    publish_point_data.markers.append(path_point_msg)
    publish_point_data.markers.append(trans_path_point_msg)
    trajectoryMarkerPub.publish(publish_point_data)

def main():
    rospy.init_node("create_dataset", anonymous=True)
    rospy.loginfo("node start!")

    srv = rospy.Service('/rtabmap_create_data/create_dataset', CreateData, generateData)
    rospy.spin()

if __name__ == "__main__":
    main()