#!/usr/bin/env python3
import rospy
import message_filters
import tf
import os
import sys
sys.path.append('/home/kal3/kal3_ws/src/kal3_perception/src/kal3_perception_mapping/src')

from jsk_recognition_msgs.msg import BoundingBoxArray
from sensor_msgs.msg import Image, CameraInfo
from visualization_msgs.msg import Marker
from lib.utils import sign_pose_2_marker_msg, bbox2str, check_bbox
from std_msgs.msg import Float64
from geometry_msgs.msg import PoseArray,Pose

from lib import deprojection
from lib.map import Map
import numpy
import argparse


class MappingNode:
    def __init__(self):
        depth_subscriber = message_filters.Subscriber("/camera_front/aligned_depth_to_color/image_raw", Image)
        caminfo_subscriber = message_filters.Subscriber("/camera_front/aligned_depth_to_color/camera_info", CameraInfo)
        yolo_subscriber = message_filters.Subscriber("/kal_3/trafficsigndetect/prediction/raw", BoundingBoxArray)
        self.sensor_subscriber_yolo = message_filters.ApproximateTimeSynchronizer(
            [yolo_subscriber, depth_subscriber, caminfo_subscriber],slop=0.1,
            queue_size=10
        )
        self.sensor_subscriber_yolo.registerCallback(self.sensor_callback)
        self.marker_publisher = rospy.Publisher('sign_markers', Marker, queue_size=10)
        self.sign_publisher = rospy.Publisher('sign_dist',Float64,queue_size=10)
        self.cone_dist_publisher =rospy.Publisher('cone_dist',Float64,queue_size=10)
        self.cone_publisher = rospy.Publisher('cone_position',PoseArray,queue_size=10)
        self.tl = tf.TransformListener()
        self.tf_exception_counter = 0

        self.map = Map()

    def sensor_callback(self, bounding_boxes, depthimg_msg, caminfo_msg):
        """
        Function to handle incoming BoundingBoxArray messages from the street sign detector.
        """
        #rospy.loginfo(f"received Bboxarray with {len(bounding_boxes.boxes)} boxes")
        sign_dist=100
        cone_dist_init=100
        cones_position = PoseArray()
        conelist=[]
        conelist.append(cone_dist_init)
        conelist2=[]
        cones_position.header.frame_id = "cones_position_frame"
        cones_position.header.stamp = rospy.Time.now()
        #print("ok")
        for bbox in bounding_boxes.boxes:
            if not check_bbox(bbox, caminfo_msg):
                rospy.logwarn(f"Ignoring weird bbox: {bbox2str(bbox)}")
                continue

            try:
                sign_pose_cam,sign_dist,cone_dist= deprojection.get_relative_pose_from_bbox(bbox, caminfo_msg,
                                                                         depthimg_msg, is_cone=(bbox.label == 1))
                #print("stop sign distance is:",distance_to_stopsign)
                if bbox.label ==1:
                    cone_position = Pose(sign_pose_cam.pose.position,sign_pose_cam.pose.orientation)
                    cones_position.poses.append(cone_position)
                    conelist.append(cone_dist)
                    if cone_dist < 3:
                        conelist2.append(cone_dist)
                
            except SystemError as e:
                if "LinAlgError" in str(e):
                    rospy.logwarn(f"Encountered linalg error while trying to get orientation"
                                  + f"from sign: {bbox2str(bbox)}")
                    continue
                else:
                    raise
            
            sign_pose_cam.header.frame_id = "camera_front_color_optical_frame"
            try:
                sign_pose_world = self.tl.transformPose("stargazer", sign_pose_cam)
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                # if no transform can be found, skip (when testing this only occured at the
                # begining of bag playback
                self.tf_exception_counter += 1
                rospy.loginfo("Encountered exception while looking up transform"
                              + f"({self.tf_exception_counter} encountered so far)")
                continue

            self.map.add_observation(sign_pose_world, sign_type=bbox.label)
            # TODO: don't publish so often?
            self.map.publish_markers(self.marker_publisher, bbox.header.stamp)
        # Method1: get 4 nearest cone's position
        if len(conelist2)>=2:
            min_indices = sorted(range(len(conelist2)), key=lambda i: conelist2[i])[:2]
        else:
            min_indices = sorted(range(len(conelist2)), key=lambda i: conelist2[i])

        if len(conelist)!=0:
            cone_dist = min(conelist)
        self.sign_publisher.publish(sign_dist)
        self.cone_dist_publisher.publish(cone_dist)

        # # Method 2: get nearest cone and furthest cone's position
        # if len(conelist2)!=0:
        #     #print(conelist2)
        #     min_indices=conelist2.index(min(conelist2))
        #     max_indices=conelist2.index(max(conelist2))
        #     newcones=PoseArray()
        #     newcones.header.frame_id = "camera_front_depth_optical_frame"
        #     newcones.header.stamp = rospy.Time.now()
        #     if min_indices!=max_indices:
        #         newcones.poses.append(cones_position.poses[min_indices])
        #         newcones.poses.append(cones_position.poses[max_indices])
        #     else:
        #         newcones.poses.append(cones_position.poses[min_indices])
        #     self.cone_publisher.publish(newcones)
        # # publish the position of the cones
        newcones=PoseArray()
        newcones.header.frame_id = "camera_front_color_optical_frame"
        newcones.header.stamp = rospy.Time.now()
        for i in range(len(min_indices)):
            newcones.poses.append(cones_position.poses[min_indices[i]])
        self.cone_publisher.publish(newcones)

if __name__ == "__main__":
    # TODO: if we use a launch file, we maybe have to set the argparse differently
    # https://discourse.ros.org/t/getting-python-argparse-to-work-with-a-launch-file-or-python-node/10606
    rospy.init_node("mapping_node", log_level=rospy.INFO)
    rospy.loginfo("Starting mapping node...")
    rospy.loginfo("Mapping node started.")
    mapping_node = MappingNode()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Received keyboard interrupt, shutting down...")
    rospy.loginfo("Mapping node finished.")
