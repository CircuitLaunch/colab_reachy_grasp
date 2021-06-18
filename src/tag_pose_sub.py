#!/usr/bin/env python3  
import rospy
import tf_conversions
import tf2_ros
import geometry_msgs.msg
import turtlesim.msg
from apriltag_ros.msg import AprilTagDetection, AprilTagDetectionArray

#build a tf broadcast message based on subscriber to the tag_detections topic that apriltag publishes to
def handle_pose(msg):
    br = tf2_ros.TransformBroadcaster()

    for a in msg.detections:

        t = geometry_msgs.msg.TransformStamped()
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = "camera_link_optical"
        t.child_frame_id = str(a.id[0])
        t.transform.translation.x = a.pose.pose.pose.position.x
        t.transform.translation.y = a.pose.pose.pose.position.y
        t.transform.translation.z = a.pose.pose.pose.position.z
        t.transform.rotation.x = a.pose.pose.pose.orientation.x
        t.transform.rotation.y = a.pose.pose.pose.orientation.y
        t.transform.rotation.z = a.pose.pose.pose.orientation.z
        t.transform.rotation.w = a.pose.pose.pose.orientation.w

        br.sendTransform(t)

if __name__ == '__main__':
    rospy.init_node('tag_pose_sub')
    rospy.Subscriber('tag_detections',
                     AprilTagDetectionArray,
                     handle_pose)
    rospy.spin()