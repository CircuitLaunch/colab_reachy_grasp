#!/usr/bin/env python3
import sys
import tf2_ros
from apriltag_ros.msg import AprilTagDetectionArray
from geometry_msgs.msg import PoseStamped, Pose
import geometry_msgs.msg
import rospy
import time

class Apriltag_Converter():

    def __init__(self):
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)

    def apriltag_callback(self, msgs):
        rate = rospy.Rate(5.0)
        
        try:
            self.trans = self.tfBuffer.lookup_transform('pedestal', 'apriltag_5',rospy.Time(0))
            # rospy.loginfo(self.trans)
            cube_pose = geometry_msgs.msg.Pose()
            cube_pose.position = self.trans.transform.translation
            cube_pose.orientation = self.trans.transform.rotation

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as ex:
            rospy.loginfo(tf2_ros.LookupException)
            rate.sleep()

if __name__ == "__main__":  
    try:

        rospy.init_node('apriltag_pose_pub',anonymous=True)
        apriltag_converter = Apriltag_Converter()

        print('hola')
        rospy.loginfo("in main")

        # rospy.init_node('tag_detection_to_tf')
        rospy.Subscriber('/tag_detections',AprilTagDetectionArray,apriltag_converter.apriltag_callback)

        # Add more publishers if needed
        # cubeATPub = rospy.Publisher('cubePose',PoseStamped, queue_size=10)
        # reachyATPub = rospy.Publisher('reachyPose', PoseStamped, queue_size=10)
        cubeATPub = rospy.Publisher('cubePose',Pose, queue_size=10)
        # reachyATPub = rospy.Publisher('reachyPose', Pose, queue_size=10)

        rospy.spin()


    except rospy.ROSInterruptException:
        pass
