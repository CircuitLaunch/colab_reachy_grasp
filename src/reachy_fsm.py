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
import copy
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
        self.a = True
        #TODO: the move_interface object should be passed among extend and grasp 

    def _cube1_callback(self, msg):
        rospy.loginfo("Found a cube")
        #TODO: keep updating the pose of the cube
        with self._mutex:
            # self.pose = msg.pose
            self.mo.update_cube_pose()

    def execute(self, userdata):
        rospy.loginfo("Executing APPROACH State")
        cubeSub = rospy.Subscriber('cubePose', PoseStamped, self._cube1_callback)

        if self.flag:       
            self.flag = False
            self.time = rospy.get_time()

        '''
        #TODO: outmost layer: check if the trj is in progress
        if trj is NOT in progress
            go to goal and set the local approach pose to be the same as approach pose
        if trj is in progress
            check if the local pose and the appproach pose has changed. if it changes then that
            means that the cube pose has changed. therefore i need to abort trj 
        keep checking if the distance between current pose and the goal is within boundary
        '''
        local_approach_pose = copy.deepcopy(self.approach_pose)


        #TODO: 1. Find if there's a cube in sight
        #      2. If the cube position changes by a lot, then run this state again -> will need to keep track of the previous cube pose    
        while True:
            #TODO: keep checking if the cube callback has been changed by a threshold. BUT
            # This MUST be done once the plan has started


            # keep updating the cube poses
            # self.mo.update_cube_pose()
            # local_approach_pose = copy.deepcopy(self.approach_pose)

            # if trj is not in progress, go to the local_approach pose(which is set to be the same as approach pose at init) 
            if not self.mo.trajectory_in_progress:
                self.mo.go_to_pose(local_approach_pose)
                # local_approach_pose = copy.deepcopy(self.approach_pose)

            else:
                if self.mo.distance(self.mo.approach_pose, local_approach_pose) <= self.mo.min_approach_error:
                    rospy.loginfo("target has moved. aborting current trj and approaching new goal")
                    # abort current trajectory
                    self.abort_trajectory()
                    # trigger new trajectory
                    local_approach_pose = copy.deepcopy(self.mo.approach_pose)
                    self.mo.go_to_pose(local_approach_pose)

            # check if the arm is within the threshold which means it's near the goal
            if self.mo.distance(self.mo.current_pose, local_approach_pose) <= self.mo.min_approach_error:
                # approach_pose_attained = True
                self.trajectory_in_progress = False
                return 'target_locked' # move to extend state
                

            if rospy.get_time() - self.time > 20:
                self.flag = True
                return 'rest' 
            
            if self.preempt_requested():
                rospy.loginfo("preempt triggered")
                return 'preempted'

            self.rate.sleep()


            # #TODO: get a pose from the callback and then update the metadata. Then unsubscribe to the topic so that it won't be triggered
            # rospy.loginfo("outside loop")
            # # Check if cube is detected from the camera
            # if self.pose:

            #     if self.a:
            #         rospy.loginfo(self.mo.update_cube_pose())
            #         self.a = False
            #         cubeSub.unregister()
            #         print("unsubscribed from cube topic")
            #     rospy.loginfo("copy")
            #     rospy.loginfo(self.mo.approach_pose)
            #     # rospy.loginfo( self.mo.cube_pose)
            #     # tranform = userdata.approach_in.pose_transform('pedestal','apriltag_4')
            #     # rospy.loginfo(tranform)

            #     # if the trj is not in progress, it'll set the approach goal to a new variable
            #     if not self.mo.trajectory_in_progress:
            #         self.mo.go_to_pose()
            #         local_approach_pose = copy.deepcopy(self.approach_pose)
            #     # if the trj is in progress, it'll keep checking if the cube location has changed
            #     else:
            #         if self.mo.distance(self.mo.approach_pose, local_approach_pose) <= self.mo.min_approach_error:
            #             rospy.loginfo("target has moved. aborting current trj and approaching new goal")
            #             # abort current trajectory
            #             self.abort_trajectory()
            #             # trigger new trajectory
            #             local_approach_pose = copy.deepcopy(self.mo.approach_pose)
            #             self.mo.go_to_pose(local_approach_pose)

            # #TODO: check if the tfj is in progress. If yes, then keep monitoring if the dist is close
            # # if the trj is NOT in progress, check if the

            #     #TODO: check if the trj is in progress. 

            #     #TODO: check if the cube pose has changed by a lot. If so, run the approach state again

            #     #TODO: check if the current pose (from Moveit) is within certain threshold compared to the goal pose
            #     if self.mo.distance(self.mo.current_pose, local_approach_pose) <= self.mo.min_approach_error:
            #         approach_pose_attained = True
            #         self.trajectory_in_progress = False
            #         return 'target_locked' # move to extend state
                
            #     #TODO:I need the local approach pose to keep track
                
            # else:
            #     # If it doesn't detect in 20 seconds, it'll go back to rest position

            #     if rospy.get_time() - self.time > 20:
            #         self.flag = True
            #         return 'rest' 
                
            # if self.preempt_requested():
            #     rospy.loginfo("preempt triggered")
            #     return 'preempted'

            # self.rate.sleep()

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
        smach.StateMachine.add('MOVETOREACHYHOME', MoveToReachyHome(move_interface_object), 
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