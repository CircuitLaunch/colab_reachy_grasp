#!/usr/bin/env python

import rospy
import smach
from threading import Lock
from apriltag_ros.msg import AprilTagDetection, AprilTagDetectionArray

# define state Approach
class Approach(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['no_target_detected','target_detected'])
        self._mutex = Lock()

    def execute(self, userdata):
        rospy.loginfo('Executing state FOO')
        # if self.counter < 3:
        #     self.counter += 1
        #     return 'outcome1'
        # else:
        #     return 'outcome2'


# define state Idle. 
class Idle(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['Approach'])
        rospy.Subscriber('/tag_detections',
                         AprilTagDetectionArray,
                         self._idle_callback)
        self.data = None
        self._mutex = Lock()

    def _idle_callback(self, msg):
        with self._mutex:
            self.data = msg.detections

        while not rospy.is_shutdown():
            with self._mutex:
                for aprilTag in msg.detections:
                    if aprilTag.id[0] == 5:
                        # go to approach
                        pass


    
    def execute(self, userdata):
        rospy.loginfo('Executing state BAR')
        return 'outcome2'
    

# define state Align. 
class Align(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome2'])

    def execute(self, userdata):
        rospy.loginfo('Executing state BAR')
        return 'outcome2'
        

# define state Grasp. This is actually moving the gripper to hold the cube
class Grasp(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome2'])

    def execute(self, userdata):
        rospy.loginfo('Executing state BAR')
        return 'outcome2'
        

# define state RestArm. This will rest the arm 
class RestArm(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome2'])

    def execute(self, userdata):
        rospy.loginfo('Executing state BAR')
        return 'outcome2'


# main
def main():
    rospy.init_node('smach_example_state_machine')

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['outcome4', 'outcome5'])

    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('FOO', Foo(), 
                               transitions={'outcome1':'BAR', 
                                            'outcome2':'outcome4'})
        smach.StateMachine.add('BAR', Bar(), 
                               transitions={'outcome2':'FOO'})

    # Execute SMACH plan
    outcome = sm.execute()


if __name__ == '__main__':
    main()