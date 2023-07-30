#!/usr/bin/env python3
from sensor_msgs.msg import Image
from typing import List
import rospy

# TODO: declare the dependencies in the CMakeList file
from jsk_recognition_msgs.msg import BoundingBox
from pyrealsense2 import intrinsics, rs2_deproject_pixel_to_point

from cv_bridge import CvBridge
import numpy as np

def compute_lower_distance(depth_image: Image, bbox: BoundingBox) -> float:

    bridge = CvBridge()
    image = bridge.imgmsg_to_cv2(depth_image, desired_encoding='passthrough')

    width = int(bbox.dimensions.x)
    height = int(bbox.dimensions.y)

    x = int(bbox.pose.position.x)
    y = int(bbox.pose.position.y)

    mid_x = int(x+width/2)
    low_y =int(y+height*0.7)
    # addressing is flipped here because it is a matrix
    roi = image[low_y-20:low_y+20, mid_x-30:mid_x+30]


    distance_mm = np.median(roi)
    distance = distance_mm / 1000

    return distance







def compute_median_distance(depth_image: Image, bbox: BoundingBox) -> float:

    bridge = CvBridge()
    image = bridge.imgmsg_to_cv2(depth_image, desired_encoding='passthrough')

    width = int(bbox.dimensions.x)
    height = int(bbox.dimensions.y)

    x = int(bbox.pose.position.x)
    y = int(bbox.pose.position.y)

    # addressing is flipped here because it is a matrix
    roi = image[y:y+height, x:x+width]
    roi_filtered = roi[roi > 10]
    roi_filtered = roi_filtered[roi_filtered < 3000]
    distance_mm = np.median(roi_filtered)
    distance = distance_mm / 1000

    return distance


def compute_midpoint_distance(depth_image: Image, bbox: BoundingBox) -> float:

    bridge = CvBridge()
    image = bridge.imgmsg_to_cv2(depth_image, desired_encoding='passthrough')

    width = int(bbox.dimensions.x)
    height = int(bbox.dimensions.y)

    x = int(bbox.pose.position.x)
    y = int(bbox.pose.position.y)

    mid_x = int(x+width/2)
    mid_y =int(y+height/2)
    # addressing is flipped here because it is a matrix
    roi = image[mid_y,mid_x]

    distance_mm = np.median(roi)
    distance = distance_mm / 1000

    return distance

def compute_closest_distance(depth_image: Image, bbox: BoundingBox) -> float:

    bridge = CvBridge()
    image = bridge.imgmsg_to_cv2(depth_image, desired_encoding='passthrough')

    width = int(bbox.dimensions.x)
    height = int(bbox.dimensions.y)

    x = int(bbox.pose.position.x)
    y = int(bbox.pose.position.y)

    # addressing is flipped here because it is a matrix
    roi = image[y:y+height, x:x+width]

    # only take pixels that are further away than 10mm
    roi_filtered = roi[roi > 10]
    #roi_filtered = roi_filtered[roi_filtered  < 3000]
    # find the closest pixels and discard everything that is further away than
    # closest + 50mm
    closest = np.min(roi_filtered)
    roi_filtered = roi_filtered[roi_filtered < closest+50]

    distance_mm = np.median(roi_filtered)
    distance = distance_mm / 1000

    return distance

def compute_distance_scan(depth_image: Image, bbox: BoundingBox) -> List[float]:
    bridge = CvBridge()
    depth_image = bridge.imgmsg_to_cv2(depth_image, desired_encoding='passthrough')

    width = int(bbox.dimensions.x)
    height = int(bbox.dimensions.y)

    x = int(bbox.pose.position.x)
    y = int(bbox.pose.position.y)

    # addressing is flipped here because it is a matrix
    roi = depth_image[y:y+height, x:x+width]

    distance_mm = np.median(roi, axis=0)
    distance = distance_mm / 1000

    return distance


def compute_sign_orientation(
        depth_image: Image, bbox: BoundingBox, cam_intrinsics: intrinsics
) -> float:
    
    distance_scan = compute_distance_scan(depth_image, bbox)

    center_y = bbox.pose.position.y + (bbox.dimensions.y / 2)
    leftmost_x = bbox.pose.position.x
    bbox_width = int(bbox.dimensions.x)

    assert len(distance_scan) == bbox_width

    center_px_row = [[leftmost_x + x_inc, center_y] for x_inc in range(bbox_width)]

    deprojected_center_row = [
        rs2_deproject_pixel_to_point(cam_intrinsics, px, dist)
        for px, dist in zip(center_px_row, distance_scan)
    ]

    # get coordinates from realsense convention to ros convention (right-hand rule)
    rs2_x, rs2_y, rs2_z = np.array(deprojected_center_row).T
    x = rs2_z
    y = -rs2_x
    # don't need z, we work with x,y only
    # z = -rs2_y

    # fit a line y' = a x + b through the detected points on the sign
    a, b = np.polyfit(x, y, deg=1)

    # get the tangent vector originating from the left edge of the sign
    x1 = x[0]
    x2 = x[-1]
    y1 = a * x1  # +b cancels out below, so we drop it
    y2 = a * x2
    tangent = np.array([x2 - x1, y2 - y1])
    # now rotate by 90 degree clockwise (because we have the tangent vector originating from
    # the left side of the sign and want a normal vector pointing towards the camera)
    rot90 = np.array(
        [[0,  1],
         [-1, 0]]
    )
    normal = rot90 @ tangent

    theta = np.math.atan2(normal[1], normal[0])

    return theta
