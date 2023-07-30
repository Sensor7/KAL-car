#!/usr/bin/env python3
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import CameraInfo, Image
from lib.depth_processing import compute_median_distance, compute_sign_orientation, compute_closest_distance,compute_midpoint_distance,compute_lower_distance
import rospy

from tf.transformations import quaternion_from_euler

# TODO: declare the dependencies in the CMakeList file
from jsk_recognition_msgs.msg import BoundingBox
from pyrealsense2 import intrinsics, distortion, rs2_deproject_pixel_to_point


def get_relative_pose_from_bbox(
        bbox: BoundingBox, cam_info: CameraInfo, depth_image: Image, is_cone: bool = False
) -> PoseStamped:
    center_pixel = [
        bbox.pose.position.x + (bbox.dimensions.x // 2),
        bbox.pose.position.y + (bbox.dimensions.y // 2),
    ]

    # Transform camera information into pyrealsense2 format
    pr2_intrinsics = intrinsics()
    pr2_intrinsics.width = cam_info.width
    pr2_intrinsics.height = cam_info.height
    pr2_intrinsics.ppx = cam_info.K[2]
    pr2_intrinsics.ppy = cam_info.K[5]
    pr2_intrinsics.fx = cam_info.K[0]
    pr2_intrinsics.fy = cam_info.K[4]
    pr2_intrinsics.coeffs = cam_info.D

    sign_dist=100
    cone_dist=100

    if cam_info.distortion_model == "plumb_bob":
        pr2_intrinsics.model = distortion.brown_conrady
    else:
        raise NotImplementedError(
            f"CameraInfo message specified unknown distortion model: {cam_info.distortion_model}"
        )

    if is_cone:
        # for cones, use a different method to get the avg distance
        depth = compute_median_distance(depth_image, bbox)
        cone_dist=depth
        #print(cone_dist)
    else:
        depth = compute_closest_distance(depth_image, bbox)
        sign_dist=depth

    # call to librealsense2 to do the actual deprojection
    pose_x, pose_y, pose_z = rs2_deproject_pixel_to_point(
        pr2_intrinsics, center_pixel, depth
    )
    #print(pose_x, pose_y, pose_z)
    if is_cone:
        # traffic cones don't need rotation
        yaw = 0
    else:
        yaw = compute_sign_orientation(depth_image, bbox, pr2_intrinsics)

    # use a stamped pose message as an easy way to pass on the timestamp and for possibly
    # publishing as a debug message.
    sign_pose = PoseStamped()
    sign_pose.header.stamp = bbox.header.stamp
    sign_pose.header.frame_id = cam_info.header.frame_id

    # convert coordinate system to the default ROS way (right-handed).
    sign_pose.pose.position.x = pose_z
    sign_pose.pose.position.y = -pose_x
    sign_pose.pose.position.z = -pose_y

    qx, qy, qz, qw = quaternion_from_euler(0, 0, yaw)
    sign_pose.pose.orientation.x = qx
    sign_pose.pose.orientation.y = qy
    sign_pose.pose.orientation.z = qz
    sign_pose.pose.orientation.w = qw
    return sign_pose, sign_dist,cone_dist
