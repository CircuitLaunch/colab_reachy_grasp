#!/usr/bin/env python

import rospy
import smach
from threading import Lock
from apriltag_ros.msg import AprilTagDetectionArray
from geometry_msgs.msg import PoseStamped


# define state Idle. 
class Idle(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['no_target_detected','target_detected','preempted'])
        rospy.Subscriber('/tag_detections',
                         AprilTagDetectionArray,
                         self._idle_callback)
        self.data = None
        self._mutex = Lock()

    def _idle_callback(self, msg):
        with self._mutex:
            self.data = msg.detections

        # while not rospy.is_shutdown():
        #     with self._mutex:
        #         for aprilTag in msg.detections:
        #             if aprilTag.id[0] == 5:
        #                 # go to approach
        #                 pass


    
    def execute(self, userdata):
        rospy.loginfo('Executing state BAR')
        return 'preempted'
    
# define state Approach
class Approach(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['target_changed','target_locked','preempted'])
        self._mutex = Lock()

    def execute(self, userdata):
        rospy.loginfo('Executing state FOO')


# define state Extend. 
class Extend(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['target_changed','target_locked','preempted',])

    def execute(self, userdata):
        rospy.loginfo('Executing state BAR')
        return 'preempted'
        

# define state Grasp. This is actually moving the gripper to hold the cube
class Grasp(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['finished','preempted'])

    def execute(self, userdata):
        rospy.loginfo('Executing state BAR')
        return 'preempted'


# main
def main():
    rospy.init_node('smach_example_state_machine')

    # Subscribe to apriltags (These should go under moveit_interface)
    # rospy.Subscriber('cubePose',PoseStamped, cube_pose_callback)
    # rospy.Subscriber('reachyPose',PoseStamped, reachy_pose_callback)
        

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['exit'])

    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('IDLE', Idle(), 
                               transitions={'no_target_detected':'IDLE', 
                                            'target_detected':'APPROACH',
                                            'preempted': 'exit'})
        smach.StateMachine.add('APPROACH', Approach(), 
                               transitions={'target_changed':'APPROACH',
                                            'target_locked':'EXTEND',
                                            'preempted': 'exit'})
        smach.StateMachine.add('GRASP', Grasp(), 
                               transitions={'finished':'exit',
                                            'preempted': 'exit'})

    # Execute SMACH plan
    outcome = sm.execute()


if __name__ == '__main__':
    main()