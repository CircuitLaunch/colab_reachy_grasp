#!/usr/bin/env python3
import sys
import tf
from apriltag_ros.msg import AprilTagDetectionArray
from geometry_msgs.msg import PoseStamped
import rospy

def tf_handle(msgs):

    for msg in msgs.detections:

        # hardware
        # msg.pose.header.frame_id = "apriltag_" + str(msg.id[0])
        # simulation
        # msg.pose.header.frame_id = str(msg.id[0])

        # rospy.loginfo(msg)
        
        # br = tf.TransformBroadcaster()
        # br.sendTransform((msg.pose.pose.pose.position.x,
        #                   msg.pose.pose.pose.position.y,
        #                   msg.pose.pose.pose.position.z),
        #                  (msg.pose.pose.pose.orientation.x,
        #                   msg.pose.pose.pose.orientation.y,
        #                   msg.pose.pose.pose.orientation.z,
        #                   msg.pose.pose.pose.orientation.w),
        #                   rospy.Time.now(),
        #                   msg.pose.header.frame_id,
        #                   "camera_link")
        #                 #   "camera_link_optical") for simluation

        poseStamped = PoseStamped()
        poseStamped.header.stamp = rospy.Time.now()
        poseStamped.pose.position.x = msg.pose.pose.position.x
        poseStamped.pose.position.y = msg.pose.pose.position.y
        poseStamped.pose.position.z = msg.pose.pose.position.z
        poseStamped.pose.orientation.x = msg.pose.pose.orientation.x
        poseStamped.pose.orientation.y = msg.pose.pose.orientation.y
        poseStamped.pose.orientation.z = msg.pose.pose.orientation.z

        while cubeATPub.get_num_connections() <1 or reachyATPub.get_num_connections() <1:
            rospy.loginfo("waiting for the publishers to be active")

        if msg.id[0] == 4: # reachy chest
            poseStamped.header.frame_id = 'reachy_chest'
            cubeATPub.publish(poseStamped)
    
        elif msg.id[0] == 5: # cube
            poseStamped.header.frame_id = 'cube'
            reachyATPub.publish(reachyATPub)
 

if __name__ == "__main__":  
    try:
        rospy.init_node('tag_detection_to_tf')
        rospy.Subscriber('/tag_detections',AprilTagDetectionArray,tf_handle)

        # Add more publishers if needed
        cubeATPub = rospy.Publisher('cubePose',PoseStamped, queue_size=10)
        reachyATPub = rospy.Publisher('reachyPose', PoseStamped, queue_size=10)

        rospy.spin()

    except rospy.ROSInterruptException:
        pass
