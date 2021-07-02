#!/usr/bin/env python3

import rospy
from rospy.core import rospyinfo
from rospy.impl.transport import Transport
import smach
from threading import Lock, Thread
from apriltag_ros.msg import AprilTagDetectionArray
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
import time
import move_interface

RATE = 1
# define state Idle. 
class Idle(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['reachyHome',
                                             'preempted'])

        self.data = None
        self.rate = rospy.Rate(RATE) # 10hz
        self._mutex = Lock()

        rospy.Subscriber('state_transition', String, self._idle_callback)


    def _idle_callback(self, msg):
        rospy.loginfo("IN IDLE CALLBACK")
        with self._mutex:
            self.data = msg.data
    
    def execute(self, userdata):
        rospy.loginfo('Executing IDLE State')

        while True:
            # rospy.loginfo(self.data)
            with self._mutex:
                if self.data == "start":
                    print(self.data)
                    self.data = None
                    return 'reachyHome'

            if self.preempt_requested():
                rospy.loginfo("preempt triggered")
                return 'preempted'
                
            self.rate.sleep()

       


# define state Ready
# class Ready(smach.State):
#     def __init__(self):
#         smach.State.__init__(self, outcomes=['preempted','relax','target_detected'])
#         self._mutex = Lock()
#         self.rate = rospy.Rate(RATE) # 10hz
#         self.pose = None
#         self.flag = True
#         # rospy.Subscriber('state_transition', String, self._state_transition)
#         self.data = None
   
#     def _ready_callback(self, msg):
#         rospy.loginfo("IN READY CALLBACK")
#         print(self.pose)
#         with self._mutex:
#             self.pose = msg.pose

#     def _state_transition(self,msg):
#         rospy.loginfo("IN READY CALLBACK, state transition")
#         with self._mutex:
#             self.data = msg.data

#     def execute(self, userdata):
#         rospy.Subscriber('cubePose', PoseStamped, self._ready_callback)
       
#         #TODO: If it can't find the apriltag for a specific amount of time, go to rest
#         rospy.loginfo("Executing READY State")
#         while True:
#             if self.flag:
#                 self.flag = False
#                 self.time = rospy.get_time()

#             with self._mutex:
#                 #works thru transition message from subscriber and switches states
#                 if self.data == "idle":
#                     self.request = None
#                     # self.goto_client.cancel_all_goals()
#                     return 'idle'

#             #TODO: Check if the apriltag (cube) is present
#             if self.pose:
#                 #TODO: Move the arm to the ready pose and send it to movegroup
#                 print(self.pose)
#                 return 'target_detected'
#             else:
#                 # If it doesn't detect in 20 seconds, it'll go back to rest position
#                 print(rospy.get_time())
#                 print(self.time)
#                 if rospy.get_time() - self.time > 20:
#                     self.flag = True
#                     return 'relax' 

#             if self.preempt_requested():
#                 rospy.loginfo("preempt triggered")
#                 return 'preempted'

#             self.rate.sleep()

# define state Rest
class Rest(smach.State):
    def __init__(self,mo):
        smach.State.__init__(self, outcomes=['preempted',
                                             'idle'])
        self._mutex = Lock()
        self.mo = mo
        self.rate = rospy.Rate(RATE) # 10hz
        rospy.Subscriber('state_transition', String, self._state_transition)

    def _state_transition(self,msg):
        rospy.loginfo("IN REST CALLBACK, state transition")
        with self._mutex:
            self.data = msg.data

    def execute(self, userdata):
        rospy.loginfo("Executing REST State")
        while True:

            #TODO: Move the robot back to arm rest position


            if self.preempt_requested():
                rospy.loginfo("preempt triggered")
                return 'preempted'

            self.rate.sleep()




# define state Approach
class Approach(smach.State):
    def __init__(self, mo):
        smach.State.__init__(self, outcomes=['target_changed',
                                             'target_locked',
                                             'reachyHome',
                                             'preempted'])
        self._mutex = Lock()
        self.mo = mo
        self.rate = rospy.Rate(RATE) # 10hz
        self.flag = True
        self.pose = None

        #TODO: the move_interface object should be passed among extend and grasp 

    def _cube1_callback(self, msg):
        rospy.loginfo("Found a cube")
        with self._mutex:
            self.pose = msg.pose

    def execute(self, userdata):
        rospy.loginfo("Executing APPROACH State")
        cubeSub = rospy.Subscriber('cubePose', PoseStamped, self._cube1_callback)

        if self.flag:       
            self.flag = False
            self.time = rospy.get_time()
                
        while True:
            #TODO: get a pose from the callback and then update the metadata. Then unsubscribe to the topic so that it won't be triggered
            if self.pose:
                trans = self.tfBuffer.lookup_transform('pedestal', 'apriltag_5',rospy.Time(0))  #apriltag5 is on the cube
                rospy.loginfo(trans)
                # save the cube pose to the object
                rospy.loginfo("cube pose")
                # rospy.loginfo( self.mo.cube_pose)
                # userdata.approach_out = userdata.approach_in
                # self.mo.cube_pose = self.pose
                cubeSub.unregister()
                print("unsubscribed from cube topic")
                rospy.loginfo("copy")
                # rospy.loginfo( self.mo.cube_pose)
                # tranform = userdata.approach_in.pose_transform('pedestal','apriltag_4')
                # rospy.loginfo(tranform)




                #TODO: create trasnform method in the move_interface.py => this will create the transform between two frames

                #TODO: 

                #TODO:

                #TODO:
                
            else:
                # If it doesn't detect in 20 seconds, it'll go back to rest position

                if rospy.get_time() - self.time > 20:
                    self.flag = True
                    return 'rest' 
                
            if self.preempt_requested():
                rospy.loginfo("preempt triggered")
                return 'preempted'

            self.rate.sleep()

# define state Extend. 
class Extend(smach.State):
    def __init__(self, mo):
        smach.State.__init__(self, outcomes=['target_changed',
                                             'target_locked',
                                             'preempted',
                                             'reachyHome'])
        self.rate = rospy.Rate(RATE) # 10hz
        self._mutex = Lock()
        self.mo = mo

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
    def __init__(self, mo):
        smach.State.__init__(self, outcomes=['apriltagHome',
                                             'preempted'])
        self.rate = rospy.Rate(RATE) # 10hz
        self._mutex = Lock()
        self.mo = mo

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

# define state Grasp. This is actually moving the gripper to hold the cube
class MoveToAprilTagHome(smach.State):
    def __init__(self, mo):
        smach.State.__init__(self, outcomes=['release',
                                             'preempted'])
        self.rate = rospy.Rate(RATE) # 10hz
        self._mutex = Lock()
        self.mo = mo

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

# define state Grasp. This is actually moving the gripper to hold the cube
class MoveToReachyHome(smach.State):
    def __init__(self, mo):
        smach.State.__init__(self, outcomes=['approach',
                                             'rest',
                                             'preempted'])
        self.rate = rospy.Rate(RATE) # 10hz
        self._mutex = Lock()
        self.flag = True
        self.pose = None
        self.mo = mo

    def execute(self, userdata):
        rospy.loginfo("Executing MoveToReachyHome State")
        
        while True:
            
            if self.preempt_requested():
                rospy.loginfo("preempt triggered")
                return 'preempted'

            #TODO: Move the arm to the ready pose and send it to movegroup
            rospy.loginfo("MOVING THE ARM TO HOME POSITION...")
            time.sleep(1)
            rospy.loginfo("MOVED THE ARM TO HOME POSITION")
            
            self.rate.sleep()

            return 'approach'


# define state Grasp. This is actually moving the gripper to hold the cube
class Release(smach.State):
    def __init__(self,mo):
        smach.State.__init__(self, outcomes=['reachyHome',
                                             'preempted'])
        self.rate = rospy.Rate(RATE) # 10hz
        self._mutex = Lock()
        self.mo = mo

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

    #TODO: create an isntance of move interface and pass it among approach, extend and grasp.
    # This is important because this object contains all the attributes of the system and must be shared 
    move_interface_object = move_interface.MoveGroupPythonInterfaceTutorial()
    # ERROR ABOUT THE ROBOT DESCRIPTION (NEED TO BE CONNECTED TO THE PHYSICAL ROBOT? TEST IT OUT ON THE REAL ROBOT)

    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('IDLE', Idle(), 
                               transitions={'reachyHome':'MOVETOREACHYHOME', 
                                            'preempted': 'exit'})
        # smach.StateMachine.add('READY', Ready(), 
        #                        transitions={'relax':'REST', 
        #                                     'target_detected':'APPROACH',
        #                                     'preempted': 'exit'})
        
        # check if there's no more apriltags to be moved. have some sort of flag that checks if the apriltag has been moved to home.
        # if everything has been moved then end the run
        smach.StateMachine.add('MOVETOREACHYHOME', MoveToReachyHome(), 
                               transitions={'approach':'APPROACH',
                                            'rest':'REST',
                                            'preempted': 'exit'})

        smach.StateMachine.add('APPROACH', Approach(move_interface_object), 
                               transitions={'target_changed':'APPROACH',
                                            'target_locked':'EXTEND',
                                            'reachyHome':'MOVETOREACHYHOME',
                                            'preempted': 'exit'})
                                # remapping={'approach_in':'move_interface_object',
                                #             'approach_in':'move_interface_object'})
                                            
        smach.StateMachine.add('EXTEND', Extend(move_interface_object), 
                               transitions={'target_changed':'APPROACH',
                                            'target_locked':'GRASP',
                                            'reachyHome':'MOVETOREACHYHOME',
                                            'preempted': 'exit'})
                                # remapping={'extend_in':'move_interface_object',
                                #             'extend_out':'move_interface_object'})
        smach.StateMachine.add('GRASP', Grasp(move_interface_object), 
                               transitions={'apriltagHome' : 'MOVETOAPRILTAGHOME',
                                            'preempted': 'exit'})
                                # remapping={'grasp_in':'move_interface_object',
                                #             'grasp_out':'move_interface_object'})
        smach.StateMachine.add('REST', Rest(move_interface_object), 
                               transitions={'idle':'IDLE', 
                                            'preempted': 'exit'})
        # will need to map the home to home pos
        smach.StateMachine.add('MOVETOAPRILTAGHOME', MoveToAprilTagHome(move_interface_object), 
                               transitions={'release':'RELEASE', 
                                            'preempted': 'exit'})

        smach.StateMachine.add('RELEASE', Release(move_interface_object), 
                                transitions={'reachyHome':'MOVETOREACHYHOME', 
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