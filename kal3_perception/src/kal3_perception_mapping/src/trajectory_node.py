import math
import rospy
from geometry_msgs.msg import PoseArray, Pose, Quaternion, PoseStamped
from nav_msgs.msg import Path
from std_msgs.msg import Bool
import tf2_ros
import tf2_geometry_msgs

class Trajectory:
    def __init__(self):
        self.leftcones = []
        self.rightcones = []
        self.detectedcones = []
        self.safedist = 0.8 #distance_y between far and near,tune this
        self.midpoint = []
        self.midpoint.append([0,0])
        self.right = False
        self.left = False
        self.midpointPublisher = rospy.Publisher(
            "trajectory_planner_node", Path, queue_size=10)
        self.coneSubscriber=rospy.Subscriber('/cone_position',PoseArray, self.sorting, queue_size=10)

# calculate distance between two points
    def calculate_distance(self,p1, p2):
        dx = p2[0] - p1[0]
        dy = p2[1] - p1[1]
        return math.sqrt(dx * dx + dy * dy)
    def calculate_distance_y(self,farcone,nearcone):
        return abs(farcone[1]-nearcone[1])
    def list_to_path(self) :
        t = rospy.Time.now()

        tf_buffer = tf2_ros.Buffer()  # tf buffer length
        tf_listener = tf2_ros.TransformListener(tf_buffer)

        transformer = tf_buffer.lookup_transform("stargazer", "vehicle_rear_axle", rospy.Time(), rospy.Duration(1.0))

        path = Path()
        path.header.frame_id = "stargazer"
        path.header.stamp = t
        assert self.midpoint is not None
        for i in range(len(self.midpoint)):
            midpoint = self.midpoint[i]
            assert type(midpoint) == list
            assert len(midpoint) == 2

            pose_to_transform = PoseStamped()
            pose_to_transform.header.stamp = t
            pose_to_transform.header.frame_id = "camera_front_color_optical_frame"
            pose_to_transform.pose.position.x = midpoint[0]
            pose_to_transform.pose.position.y = midpoint[1]
            pose_to_transform.pose.position.z = 0
            pose_to_transform.pose.orientation = Quaternion(0, 0, 0, 1)

            try:
                pose_transformed = tf2_geometry_msgs.do_transform_pose(pose_to_transform, transformer)
                #pose_transformed = tf_buffer.transform(pose_to_transform, "stargazer")
                conepose = PoseStamped()

                conepose.header.stamp = t
                conepose.pose.position.x = pose_transformed.pose.position.x
                conepose.pose.position.y = pose_transformed.pose.position.y
                conepose.pose.position.z = 0
                conepose.pose.orientation = Quaternion(0, 0, 0, 1)
                path.poses.append(conepose)
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                rospy.loginfo("Encountered an exception while looking up transformation")

        return path

    def sorting(self, input:PoseArray):
        #rospy.loginfo(f"received PoseArray with {len(input.poses)} Poses")
        self.detectedcones.clear()
        self.leftcones.clear()
        self.rightcones.clear()
        self.midpoint.clear()
        for cone in input.poses:
            cone_x = cone.position.x
            cone_y = cone.position.y#offset
            self.detectedcones.append([cone_x,cone_y])
        cone_to_leftcone = []
        cone_to_rightcone =[]
        car = [0,0]
        if len(self.detectedcones)==2:
            d = self.calculate_distance(self.detectedcones[1],self.detectedcones[0])
            if d > self.safedist: #cones are in two rows
                for cone in self.detectedcones:
                    if cone[1]<0:
                        self.rightcones.append(cone)
                    else:
                        self.leftcones.append(cone)

                    
            else:
                if self.detectedcones[0][1]<self.detectedcones[1][1]:
                    #right row
                    self.rightcones.append(self.detectedcones[0])
                    self.rightcones.append(self.detectedcones[1])
                    # for i in range(2,len(self.detectedcones)):
                    #     two_d = self.calculate_distance(self.rightcones[-1],self.detectedcones[i])
                    #     if two_d < self.safedist:
                    #         self.rightcones.append(self.detectedcones[i])
                    #     else:
                    #         self.leftcones.append(self.detectedcones[i])
                elif self.detectedcones[0][1]>self.detectedcones[1][1]:
                    self.leftcones.append(self.detectedcones[0])
                    self.leftcones.append(self.detectedcones[1])
                    # for i in range(2,len(self.detectedcones)):
                    #     two_d = self.calculate_distance(self.leftcones[-1],self.detectedcones[i])
                    #     if two_d < self.safedist:
                    #         self.leftcones.append(self.detectedcones[i])
                    #     else:
                    #         self.rightcones.append(self.detectedcones[i])
        elif len(self.detectedcones)==1:
            if self.detectedcones[0][1]<0:
                self.rightcones.append(self.detectedcones[0])
            else:
                self.leftcones.append(self.detectedcones[0])

        #generate a path
        if len(self.leftcones)==0 and len(self.rightcones)==2:
            self.right = True
            self.left = False
        elif len(self.rightcones)==0 and len(self.leftcones)==2:
            self.right = True
            self.left = False
        elif len(self.leftcones) == 1 and len(self.rightcones)==1:
            self.right = False
            self.left = False
        elif len(self.leftcones)==0 and len(self.rightcones)==1:
            self.right = True
            self.left = False
        elif len(self.rightcones)==0 and len(self.leftcones)==1:
            pass



        if len(self.leftcones)==0 and len(self.rightcones)==2:
            d_car = self.calculate_distance(self.rightcones[0],car)
            d_y = self.calculate_distance_y(self.rightcones[1],self.rightcones[0])
            if d_car > 0.3:
                midpoint_x = 0.25
                midpoint_y = d_y*0.5
                self.midpoint.append([0,0])
                self.midpoint.append([midpoint_x,midpoint_y])
            else:
                midpoint_x = 0.15
                midpoint_y = d_y*0.7
                self.midpoint.append([0,0])
                self.midpoint.append([midpoint_x,midpoint_y])
        elif len(self.rightcones)==0 and len(self.leftcones)==2:
            d_car = self.calculate_distance(self.leftcones[0],car)
            d_y = self.calculate_distance_y(self.leftcones[1],self.leftcones[0])
            if d_car > 0.3:
                midpoint_x = 0.25
                midpoint_y = -0.35
                self.midpoint.append([0,0])
                self.midpoint.append([midpoint_x,midpoint_y])
            else:
                midpoint_x = 0.15
                midpoint_y = -0.4
                self.midpoint.append([0,0])
                self.midpoint.append([midpoint_x,midpoint_y])               
        elif len(self.leftcones) == 1 and len(self.rightcones)==1:
            midpoint_x = (self.leftcones[0][0]+self.rightcones[0][0])/2
            midpoint_y = (self.leftcones[0][1]+self.rightcones[0][1])/2
            self.midpoint.append([0,0])
            self.midpoint.append([midpoint_x,midpoint_y])
        elif len(self.leftcones)==0 and len(self.rightcones)==1:
            d_car = self.calculate_distance(self.rightcones[0],car)
            if d_car < 0.3:
                midpoint_x = 0.25
                midpoint_y = -self.rightcones[0][1]*3
            else:
                midpoint_x = 0.25
                midpoint_y = -self.rightcones[0][1]
            self.midpoint.append([0,0])
            self.midpoint.append([midpoint_x,midpoint_y])
        elif len(self.rightcones)==0 and len(self.leftcones)==1:
            d_car = self.calculate_distance(self.leftcones[0],car)
            if d_car < 0.3:
                midpoint_x = 0.25
                midpoint_y = -self.leftcones[0][1]*4
            else:
                midpoint_x = 0.25
                midpoint_y = -self.leftcones[0][1]*1.5
            midpoint_x = 0.25
            midpoint_y = -self.leftcones[0][1]
            self.midpoint.append([0,0])
            self.midpoint.append([midpoint_x,midpoint_y])
        elif len(self.rightcones)==0 and len(self.leftcones)==0:
            self.midpoint.append([0,0])
        
        # elif len(self.leftcones)>len(self.rightcones):
        #     for cone in self.leftcones:
        #         midpoint_x = (cone[0]+self.rightcones[0])/2
        #         midpoint_y = (cone[1]+self.rightcones[1])/2
        #         self.midpoint.append([midpoint_x,midpoint_y])
        # elif len(self.rightcones)>len(self.leftcones):
        #     for cone in self.rightcones:
        #         midpoint_x = (cone[0]+self.leftcones[0])/2
        #         midpoint_y = (cone[1]+self.leftcones[1])/2
        #         self.midpoint.append([midpoint_x,midpoint_y])
        
        # rospy.loginfo("left:"+str(trajectory.leftcones))

        # rospy.loginfo("right:"+str(trajectory.rightcones))

        # rospy.loginfo("path:"+str(trajectory.midpoint))
        # publish a path
        path = self.list_to_path()
        #rospy.loginfo(path)
        if len(path.poses)==1:
            pass
        else:
            self.midpointPublisher.publish(path)
                
if __name__ == '__main__':
    rospy.init_node('Generate_path', anonymous=True)
    try:
        trajectory = Trajectory()
    except rospy.ROSInterruptException:
        pass
    # rospy.loginfo("leftcones:",trajectory.leftcones)
    # rospy.loginfo("rightcones:",trajectory.rightcones)
    # rospy.loginfo("path:",trajectory.midpoint)
    rospy.spin()






