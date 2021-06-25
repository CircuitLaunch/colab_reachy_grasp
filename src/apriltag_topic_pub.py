#!/usr/bin/env python3
import sys
import tf
from apriltag_ros.msg import AprilTagDetectionArray
from geometry_msgs.msg import PoseStamped
import rospy

def apriltag_callback(msgs):

    for msg in msgs.detections:

        poseStamped = PoseStamped()
        poseStamped.header.stamp = rospy.Time.now()
        poseStamped.pose.position.x = msg.pose.pose.pose.position.x
        poseStamped.pose.position.y = msg.pose.pose.pose.position.y
        poseStamped.pose.position.z = msg.pose.pose.pose.position.z
        poseStamped.pose.orientation.x = msg.pose.pose.pose.orientation.x
        poseStamped.pose.orientation.y = msg.pose.pose.pose.orientation.y
        poseStamped.pose.orientation.z = msg.pose.pose.pose.orientation.z

        # while cubeATPub.get_num_connections() <1 or reachyATPub.get_num_connections() <1:
        #     rospy.loginfo("waiting for the publishers to be active")

        if msg.id[0] == 4: # reachy chest
            poseStamped.header.frame_id = 'reachy_chest'
            reachyATPub.publish(poseStamped)
    
        elif msg.id[0] == 5: # cube
            poseStamped.header.frame_id = 'cube'
            cubeATPub.publish(poseStamped)
 

if __name__ == "__main__":  
    try:
        rospy.init_node('apriltag_pose_pub',anonymous=True)

        # rospy.init_node('tag_detection_to_tf')
        rospy.Subscriber('/tag_detections',AprilTagDetectionArray,apriltag_callback)

        # Add more publishers if needed
        cubeATPub = rospy.Publisher('cubePose',PoseStamped, queue_size=10)
        reachyATPub = rospy.Publisher('reachyPose', PoseStamped, queue_size=10)

        rospy.spin()


    except rospy.ROSInterruptException:
        pass
