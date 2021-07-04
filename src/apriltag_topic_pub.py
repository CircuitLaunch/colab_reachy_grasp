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
        pass
        # self.listener = tf2_ros.TransformListener(self.tfBuffer)

    def apriltag_callback(self, msgs):
        self.tfBuffer = tf2_ros.Buffer()
        rate = rospy.Rate(5.0)
        
        try:
            # rospy.loginfo("cb")
            self.trans = self.tfBuffer.lookup_transform('pedestal', 'apriltag_5',rospy.Time(0))
            cube_pose = geometry_msgs.msg.Pose()
            cube_pose.position = self.trans.transform.translation
            cube_pose.orientation = self.trans.transform.rotation
            # trans = self.tfBuffer.lookup_transform('pedestal', 'apriltag_5',rospy.Time(0))
            rospy.loginfo(f'wtf: {self.trans}')
            # for msg in msgs.detections:
           
                # if msg.id[0] == 5: # cube
                #     _pose = geometry_msgs.msg.Pose()
                #     _pose.position = trans.transform.translation
                #     _pose.orientation = trans.transform.rotation
                #     rospy.loginfo(_pose)
                #     # poseStamped.header.frame_id = 'cube'
                #     cubeATPub.publish(_pose)

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rate.sleep()

if __name__ == "__main__":  
    try:
        rospy.loginfo("in main")
        apriltag_converter = Apriltag_Converter()
        rospy.init_node('apriltag_pose_pub',anonymous=True)

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
