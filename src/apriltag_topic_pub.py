#!/usr/bin/env python3
import sys
import tf2_ros
from apriltag_ros.msg import AprilTagDetectionArray
from geometry_msgs.msg import PoseStamped, Pose
import geometry_msgs.msg
import rospy

class Apriltag_Converter():

    def __init__(self):
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)
        self.rate = rospy.Rate(5.0)

    def apriltag_callback(self,msgs):

        try:
            trans = self.tfBuffer.lookup_transform('pedestal', 'apriltag_5',rospy.Time(0))

            for msg in msgs.detections:

                # poseStamped = PoseStamped()
                # poseStamped.header.stamp = rospy.Time.now()
                # poseStamped.pose.position.x = msg.pose.pose.pose.position.x
                # poseStamped.pose.position.y = msg.pose.pose.pose.position.y
                # poseStamped.pose.position.z = msg.pose.pose.pose.position.z
                # poseStamped.pose.orientation.x = msg.pose.pose.pose.orientation.x
                # poseStamped.pose.orientation.y = msg.pose.pose.pose.orientation.y
                # poseStamped.pose.orientation.z = msg.pose.pose.pose.orientation.z

                # while cubeATPub.get_num_connections() <1 or reachyATPub.get_num_connections() <1:
                #     rospy.loginfo("waiting for the publishers to be active")

                # if msg.id[0] == 4: # reachy chest
                #     trans = self.tfBuffer.lookup_transform('pedestal', 'apriltag_4',rospy.Time(0))
                #     _pose = geometry_msgs.msg.Pose()
                #     _pose.position = trans.transform.translation
                #     _pose.orientation = trans.transform.rotation
                #     # poseStamped.header.frame_id = 'reachy_chest'
                #     reachyATPub.publish(poseStamped)
            
                if msg.id[0] == 5: # cube
                    _pose = geometry_msgs.msg.Pose()
                    _pose.position = trans.transform.translation
                    _pose.orientation = trans.transform.rotation
                    # poseStamped.header.frame_id = 'cube'
                    cubeATPub.publish(_pose)

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            self.rate.sleep()

if __name__ == "__main__":  
    try:
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
