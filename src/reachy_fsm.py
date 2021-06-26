#!/usr/bin/env python

import rospy
from rospy.impl.transport import Transport
import smach
from threading import Lock, Thread
from apriltag_ros.msg import AprilTagDetectionArray
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String

# define state Idle. 
class Idle(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['no_target_detected','target_detected','preempted'])

        self.data = None
        self.rate = rospy.Rate(10) # 10hz
        self._mutex = Lock()
        rospy.Subscriber('idle_mode', String, self._idle_callback)


    def _idle_callback(self, msg):
        rospy.loginfo("IN IDLE CALLBACK")
        with self._mutex:
            self.data = msg.data
        rospy.loginfo("data")
        rospy.loginfo(self.data)
    
    def execute(self, userdata):
        rospy.loginfo('Executing IDLE State')
        rospy.loginfo(self.data)

        # with self._mutex:
        #     self.data = None

        while True:
            print(self.data)
            with self._mutex:
                # print("in")
                if self.data == "start":
                    self.data = None
                    rospy.loginfo("FEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEFEF")
                    return 'target_detected'
                elif self.data == "preempt":
                    return 'preempted'
                elif self.data == "fe":
                    return 'no_target_detected'

            if self.preempt_requested():
                rospy.loginfo("preempt triggered")
                return 'preempted'
                
            self.rate.sleep()

        # if self.counter < 3:
        #         self.counter +=1
        #         return 'target_detected'
        #     elif self.counter%2:
        #         self.counter +=1
        #         return 'no_target_detected'
        #     else:
        #         return 'preestarmpted'
            # with self._mutex:
            #     self.data = msg.detections

            # while not rospy.is_shutdown():
            #     with self._mutex:
            #         for aprilTag in msg.detections:
            #             if aprilTag.id[0] == 5:
            #                 # go to approach
            #                 pass











# define state Approach
class Approach(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['target_changed','target_locked','preempted'])
        self._mutex = Lock()

    def execute(self, userdata):
        rospy.loginfo("Executing APPROACH State")
        if 2>1:
            return 'target_changed'
        elif 33>3:
            return 'target_locked'
        else:
            return 'preempted'


# define state Extend. 
class Extend(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['target_changed','target_locked','preempted',])

    def execute(self, userdata):
        rospy.loginfo("Executing EXTEND State")
        if 2>1:
            return 'target_changed'
        elif 33>3:
            return 'target_locked'
        else:
            return 'preempted'
        

# define state Grasp. This is actually moving the gripper to hold the cube
class Grasp(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['finished','preempted'])

    def execute(self, userdata):
        rospy.loginfo("Executing GRASP State")
        rospy.loginfo('Executing state BAR')
        if 4>3:
            return 'finished'
        else:
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
        smach.StateMachine.add('EXTEND', Extend(), 
                               transitions={'target_changed':'APPROACH',
                                            'target_locked':'EXTEND',
                                            'preempted': 'exit'})
        smach.StateMachine.add('GRASP', Grasp(), 
                               transitions={'finished':'exit',
                                            'preempted': 'exit'})

    # Execute SMACH plan
    smach_thread = Thread(target=sm.execute)
    smach_thread.start()

    rospy.spin()
    print("prempting")
    sm.request_preempt()

    smach_thread.join()


if __name__ == '__main__':
    main()