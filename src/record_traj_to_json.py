#!/usr/bin/env python3

#listens for a goal message being sent to a controller, and records this message to a file as a json string.
#useful for recording Moveit messages to play back later or for testing

import json
import rospkg
from rospy_message_converter import json_message_converter
#above is a ros package that you must add to your workspace and build: https://github.com/uos/rospy_message_converter

import rospy


from control_msgs.msg import (
    FollowJointTrajectoryActionGoal,
)

from trajectory_msgs.msg import (
    JointTrajectoryPoint,
)

def callback(data):

    rospy.loginfo("I heard %s", data)

    json_msg = json_message_converter.convert_ros_message_to_json(data)
    rospack = rospkg.RosPack()

    path = rospack.get_path('reachy_gazebo_grasp')

    with open( path + "/data/data_file.json", "w") as write_file:
        json.dump(json_msg, write_file, indent=4)
    
def listener():

    rospy.init_node('traj_goal_listener', anonymous=True)

    #topic on which you wish to intercept the message. This is generally one of the topics provided by your action server that MoveIt interfaces with.
    controller_topic = "/left_arm_position_controller/follow_joint_trajectory/goal"

    rospy.Subscriber(controller_topic, FollowJointTrajectoryActionGoal, callback)

    rospy.spin()

if __name__ == '__main__':
    listener()