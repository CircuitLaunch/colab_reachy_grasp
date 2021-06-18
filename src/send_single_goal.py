#!/usr/bin/env python3

#this reads in a json string from a file (of a pre-recorded Moveit trajectory), and sends it to your controllers action server.
#It is useful for sending single trajectories to your action server for general testing without having to run MoveIt.
#It can also be used to send single pre-recorded trajectories to your controllers

#based partly on: https://github.com/RethinkRobotics/baxter_examples/blob/master/scripts/joint_trajectory_client.py

import sys
import rospy
import rospkg

import actionlib

import json

from control_msgs.msg import (
    FollowJointTrajectoryAction,
    FollowJointTrajectoryGoal,
)

from trajectory_msgs.msg import (
    JointTrajectoryPoint,
)

class Trajectory(object):
    def __init__(self, controller_topic):
        self._client = actionlib.SimpleActionClient(
            controller_topic,
            FollowJointTrajectoryAction
        )
        self._goal = FollowJointTrajectoryGoal()
        self._goal.goal_time_tolerance = rospy.Time(0.1)
        server_up = self._client.wait_for_server(timeout=rospy.Duration(10.0))
        if not server_up:
            rospy.logerr("Timed out waiting for Joint Trajectory"
                         " Action Server to connect. Start the action server"
                         " before running example.")
            rospy.signal_shutdown("Timed out waiting for Action Server")
            sys.exit(1)

    #loads the data from the json file and populates the message we will later send
    #the header stamp is added later, right before the message is sent
    def load(self, data_path):
        
      with open( data_path, "r") as read_file:
        data = json.load(read_file)
      t = json.loads(data)

      self._goal.trajectory.header.frame_id = t['goal']['trajectory']['header']['frame_id']
      self._goal.trajectory.joint_names = t['goal']['trajectory']['joint_names']
      self.add_points(t['goal']['trajectory']['points'])

    #iterates thru the points array and populates the message
    def add_points(self, points):
        
      for p in points:
        point = JointTrajectoryPoint()
        point.positions = p['positions']
        point.velocities = p['velocities']
        point.accelerations = p['accelerations']
        point.time_from_start.secs = p['time_from_start']['secs']
        point.time_from_start.nsecs = p['time_from_start']['nsecs']
        self._goal.trajectory.points.append(point)

    #adds the header and sends the goal to the server
    def start(self):
        self._goal.trajectory.header.stamp = rospy.Time.now()
        self._client.send_goal(self._goal)

    def stop(self):
        self._client.cancel_goal()

    def wait(self, timeout=15.0):
        self._client.wait_for_result(timeout=rospy.Duration(timeout))

    def result(self):
        return self._client.get_result()

def main():
    """RSDK Joint Trajectory Example: Simple Action Client
    Creates a client of the Joint Trajectory Action Server
    to send commands of standard action type,
    control_msgs/FollowJointTrajectoryAction.
    Make sure to start the joint_trajectory_action_server.py
    first. Then run this example on a specified limb to
    command a short series of trajectory points for the arm
    to follow.
    """
    
    print("Initializing node... ")
    rospy.init_node("joint_trajectory_client")
    print("Getting robot state... ")
    print("Running. Ctrl-c to quit")

    controller_topic = '/left_arm_position_controller/follow_joint_trajectory'


    traj = Trajectory(controller_topic)

    #creates the path to the json file we are going to read
    rospack = rospkg.RosPack()
    path = rospack.get_path('reachy_gazebo_grasp')
    data_path = "/data/"
    data_file = "data_file.json"
    full_path = path + data_path + data_file

    traj.load(full_path)

    rospy.on_shutdown(traj.stop)

    traj.start()
    traj.wait(15.0)
    rospy.loginfo(traj.result())
    print("Exiting - Joint Trajectory Action Test Complete")

if __name__ == "__main__":
    main()