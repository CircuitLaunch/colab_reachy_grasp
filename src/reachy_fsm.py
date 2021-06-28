#!/usr/bin/env python

import rospy
from rospy.impl.transport import Transport
import smach
from threading import Lock, Thread
from apriltag_ros.msg import AprilTagDetectionArray
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String

RATE = 1
# define state Idle. 
class Idle(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['ready','preempted'])

        self.data = None
        self.rate = rospy.Rate(RATE) # 10hz
        self._mutex = Lock()

        rospy.Subscriber('idle_mode', String, self._idle_callback)


    def _idle_callback(self, msg):
        rospy.loginfo("IN IDLE CALLBACK")
        with self._mutex:
            self.data = msg.data
    
    def execute(self, userdata):
        rospy.loginfo('Executing IDLE State')

        while True:
            rospy.loginfo(self.data)
            with self._mutex:
                if self.data == "start":
                    self.data = None
                    return 'ready'


            if self.preempt_requested():
                rospy.loginfo("preempt triggered")
                return 'preempted'
                
            self.rate.sleep()

       


# define state Ready
class Ready(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['preempted','relax','target_detected'])
        self._mutex = Lock()
        self.rate = rospy.Rate(RATE) # 10hz
        self.data = None
        rospy.Subscriber('cubePose', PoseStamped, self._ready_callback)

    def _ready_callback(self, msg):
        rospy.loginfo("IN READY CALLBACK")
        with self._mutex:
            self.data = msg.data

    def execute(self, userdata):
       
        #TODO: If it can't find the apriltag for a specific amount of time, go to rest
        rospy.loginfo("Executing READY State")
        while True:

            #TODO: Check if the apriltag (cube) is present
            # if 2>1:
            #     return 'target_detected'
            # elif 33>3:
            #     return 'relax'

            if self.preempt_requested():
                rospy.loginfo("preempt triggered")
                return 'preempted'

            self.rate.sleep()

# define state Rest
class Rest(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['preempted','ready'])
        self._mutex = Lock()
        self.rate = rospy.Rate(RATE) # 10hz

    def execute(self, userdata):
        rospy.loginfo("Executing REST State")
        while True:
            if 2>1:
                return 'ready'
            elif 33>3:
                return 'target_locked'
            
            if self.preempt_requested():
                rospy.loginfo("preempt triggered")
                return 'preempted'

            self.rate.sleep()




# define state Approach
class Approach(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['target_changed','target_locked','ready','preempted'])
        self._mutex = Lock()
        self.rate = rospy.Rate(RATE) # 10hz

    def execute(self, userdata):
        rospy.loginfo("Executing APPROACH State")
        while True:
            if 2>1:
                return 'target_changed'
            elif 33>3:
                return 'target_locked'
            elif 3<5:
                return 'ready'
            
            if self.preempt_requested():
                rospy.loginfo("preempt triggered")
                return 'preempted'

            self.rate.sleep()

# define state Extend. 
class Extend(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['target_changed','target_locked','preempted','ready'])
        self.rate = rospy.Rate(RATE) # 10hz
        self._mutex = Lock()

    def execute(self, userdata):
        rospy.loginfo("Executing EXTEND State")
        while True:
            if 2>1:
                return 'target_changed'
            elif 33>3:
                return 'target_locked'
            elif 3>2:
                return 'ready'
            
            if self.preempt_requested():
                    rospy.loginfo("preempt triggered")
                    return 'preempted'
            
            self.rate.sleep()

# define state Grasp. This is actually moving the gripper to hold the cube
class Grasp(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['finished','ready','preempted'])
        self.rate = rospy.Rate(RATE) # 10hz
        self._mutex = Lock()

    def execute(self, userdata):
        rospy.loginfo("Executing GRASP State")
        while True:
            if 4>3:
                return 'finished'
            elif 3<5:
                return 'ready'

            if self.preempt_requested():
                    rospy.loginfo("preempt triggered")
                    return 'preempted'

            self.rate.sleep()
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
                               transitions={'ready':'READY', 
                                            'preempted': 'exit'})
        smach.StateMachine.add('READY', Ready(), 
                               transitions={'relax':'REST', 
                                            'target_detected':'APPROACH',
                                            'preempted': 'exit'})
        smach.StateMachine.add('APPROACH', Approach(), 
                               transitions={'target_changed':'APPROACH',
                                            'target_locked':'EXTEND',
                                            'ready':'READY',
                                            'preempted': 'exit'})
        smach.StateMachine.add('EXTEND', Extend(), 
                               transitions={'target_changed':'APPROACH',
                                            'target_locked':'GRASP',
                                            'ready':'READY',
                                            'preempted': 'exit'})
        smach.StateMachine.add('GRASP', Grasp(), 
                               transitions={'finished':'exit',
                                            'ready' : 'READY',
                                            'preempted': 'exit'})
        smach.StateMachine.add('REST', Rest(), 
                               transitions={'ready':'IDLE', 
                                            'preempted': 'exit'})

    # Execute SMACH pslan
    smach_thread = Thread(target=sm.execute)
    smach_thread.start()

    rospy.spin()
    print("prempting from main...")
    sm.request_preempt()

    smach_thread.join()


if __name__ == '__main__':
    main()