#!/usr/bin/env python

from std_msgs.msg import Header
from sensor_msgs.msg import CameraInfo, RegionOfInterest, Image
from visualization_msgs.msg import Marker
from jsk_recognition_msgs.msg import BoundingBox, BoundingBoxArray
from geometry_msgs.msg import PoseStamped
from tf.transformations import euler_from_quaternion
import rospy

import cv2
from cv_bridge import CvBridge
import numpy as np


def sign_pose_2_marker_msg(pose: PoseStamped, signtype: int, id_: int, ns: str = 'sign'):
    """
    Takes a stamped pose and a sign type and returns a marker message that can be published and
    then visualized in rviz.
    -----------
    Parameters:
        pose (PoseStamped): pose of the sign with position and orientation, header should
            specify the frame and time.
        signtype (int): Type of street sign as described below.
        id_ (int): unique id of the sign.
        ns (str): namespace (TODO: how to set this?)
    -----------
    Returns:
        marker: the Marker message
    --
    Sign type correspond to colors as follows:
    Label 0/Stop : Red
    Label 1/Priority : Pink
    Label 2/Autonomous driving : Gray
    Label 3/Traffic cone: Orange
    --
    For now uses arrow markers to indicate the orientation of the sign.
    https://wiki.ros.org/rviz/DisplayTypes/Marker#Arrow_.28ARROW.3D0.29
    """
    marker = Marker()
    marker.header.frame_id = pose.header.frame_id
    marker.header.stamp = pose.header.stamp
    marker.ns = ns  # TODO: how to set ns
    marker.id = id_
    marker.type = Marker.ARROW
    marker.action = Marker.ADD

    marker.pose = pose.pose

    # x sets the length of the arrow
    marker.scale.x = 0.05
    marker.scale.y = 0.02
    marker.scale.z = 0.02

    if signtype == 0:
        # Stop sign: red
        r, g, b = (1.00, 0.21, 0.28)
        marker.text = "STOP"
    elif signtype == 1:
        # Traffic cone: orange
        marker.type = Marker.SPHERE
        marker.scale.x = 0.10
        marker.scale.y = 0.10
        marker.scale.z = 0.10
        r, g, b = (0.87, 0.28, 0.16)
        marker.text = "CONE"
    else:
        raise ValueError(f"Unknown sign type {signtype}")

    marker.color.a = 1.0
    marker.color.r = r
    marker.color.g = g
    marker.color.b = b

    return marker


def bbox2str(bbox: BoundingBox):
    s = f"Bounding box with label {bbox.label} at pixel position: "
    s += f"x: {bbox.pose.position.x}+{bbox.dimensions.x} "
    s += f"y: {bbox.pose.position.y}+{bbox.dimensions.y}"
    return s

def check_bbox(bbox: BoundingBox, caminfo: CameraInfo) -> bool:
    img_height = caminfo.height
    img_width = caminfo.width

    upperleft = (bbox.pose.position.x, bbox.pose.position.y)
    dimensions = (bbox.dimensions.x, bbox.dimensions.y)

    if dimensions[0] < 6 or dimensions[1] < 6:
        return False
    if bbox.label < 0 or bbox.label > 3:
        return False
    if upperleft[0] < 0 or upperleft[1] < 0:
        return False
    if upperleft[0] + dimensions[0] > img_width:
        return False
    if upperleft[1] + dimensions[1] > img_height:
        return False
    return True

def pose2str(pose: PoseStamped):
    q = pose.pose.orientation
    _, _, theta = euler_from_quaternion([q.x, q.y, q.z, q.w])[2]
    theta_deg = theta/3.14159*180
    s = f"Pose with position ({pose.pose.position.x:.3f}, "
    s += f"{pose.pose.position.y:.3f}, {pose.pose.position.z:.3f}) "
    s += f"and rotation around z: {theta:.3f} rad / {theta_deg:.1f} deg"
    return s


def normalize_angle(theta: float) -> float:
    """
    normalize the angle and return a value in (-pi,pi)
    """
    while theta > np.pi:
        theta -= 2*np.pi
    while theta < -np.pi:
        theta += 2*np.pi

    return theta
