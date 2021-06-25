#!/usr/bin/env python3
import sys
import tf
from apriltag_ros.msg import AprilTagDetectionArray
from geometry_msgs.msg import Pose
import rospy

def tf_handle(msgs):

    # rate = rospy.Rate(10)
    for msg in msgs.detections:

        topicName = "apriltag_" + str(msg.id[0])
        apriltagPub = rospy.Publisher(topicName,Pose, queue_size=10)
        
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

        pose = Pose()
        pose.position.x = msg.pose.pose.pose.position.x
        pose.position.y = msg.pose.pose.pose.position.y
        pose.position.z = msg.pose.pose.pose.position.z
        pose.orientation.x = msg.pose.pose.pose.orientation.x
        pose.orientation.y = msg.pose.pose.pose.orientation.y
        pose.orientation.z = msg.pose.pose.pose.orientation.z

        while apriltagPub.get_num_connections() <1:
            rospy.loginfo("waiting for the publisher to open")
        apriltagPub.publish(pose)
        # rate.sleep()
        

if __name__ == "__main__":  
    try:
        rospy.init_node('tag_detection_to_tf')
        rospy.Subscriber('/tag_detections',AprilTagDetectionArray,tf_handle)

        rospy.spin()

    except rospy.ROSInterruptException:
        pass
