#!/usr/bin/env python3

import rospy
from rospy.core import rospyinfo
from rospy.impl.transport import Transport
import smach
from tf.transformations import *
import math
from threading import Lock, Thread
from apriltag_ros.msg import AprilTagDetectionArray
from geometry_msgs.msg import PoseStamped, Pose
from std_msgs.msg import String
from colab_reachy_control.srv import Recover, RecoverRequest, Relax, RelaxRequest, Zero, ZeroRequest
import time
import copy
import move_interface
import geometry_msgs.msg

RATE = 10
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
                    rospy.loginfo("------------------------------")
                    return 'reachyHome'

            if self.preempt_requested():
                rospy.loginfo("preempt triggered")
                return 'preempted'
                
            self.rate.sleep()

       
# define state Approach
class Approach(smach.State):
    def __init__(self, mo):
        smach.State.__init__(self, outcomes=['approach',
                                             'target_locked',
                                             'reachyHome',
                                             'preempted',
                                             'relax'])
        self._mutex = Lock()
        self.mo = mo
        self.rate = rospy.Rate(50) # 10hz
        self.flag = True
        self.pose = None
        self.a = True
        self.reachyRelax = rospy.ServiceProxy('relax', Relax)
        #TODO: the move_interface object should be passed among extend and grasp 

    def _cube1_callback(self, msg):
        self.mo.lookup_tf()

    def execute(self, userdata):
        rospy.loginfo("Executing APPROACH State")
        cubeSub = rospy.Subscriber('cubePose', PoseStamped, self._cube1_callback)
        # if self.flag:       
        #     self.flag = False
        #     self.time = rospy.get_time()
            
        self.mo.lookup_tf()
        
        local_approach_pose = geometry_msgs.msg.Pose()

        while True:

            if self.preempt_requested():
                rospy.loginfo("preempt triggered")
                relaxReq = RelaxRequest()
                relaxReq.side = "left"
                self.reachyRelax(relaxReq)
                # return 'relax'

            self.rate.sleep()
            
            local_approach_pose = copy.deepcopy(self.mo.approach_pose)
            # rospy.loginfo(local_approach_pose)
            
            #TODO: UNCOMMENT AFTER EXPERIMENT
            # result = self.mo.goToPose(local_approach_pose)
            rospy.loginfo("SIMULATING APPROACH")
            rospy.loginfo(f"-----APPROACHING {self.mo.apriltag_first_elem} -----")
            rospy.sleep(5)
            #TODO: DELTE THE BELOW. ONLY FOR SIMULATING
            result = 3

            if result == 0: # no plan
                rospy.loginfo("couldnt find a plan. attempting to find a plan again...")
                rospy.loginfo("------------------------------")
                continue
            elif result == 1: # target changed
                rospy.loginfo("target changed. attempting to find a plan again...")
                rospy.loginfo("------------------------------")
                continue
            elif result == 2: # error
                rospy.loginfo("error in actuators")
                rospy.loginfo("------------------------------")
                return "relax"
            elif result == 3: # successful
                rospy.loginfo("successfully moved to goal. changing to extend" )
                rospy.loginfo("------------------------------")
                return 'target_locked'
            else: # successfully moved but not within tolerance #TODO: This is a "minor" issue that requires calibration
                rospy.loginfo("successfully moved to goal. changing to extend" )
                rospy.loginfo("------------------------------")
                continue
            


# define state Extend. 
class Extend(smach.State):
    def __init__(self, mo):
        smach.State.__init__(self, outcomes=['approach',
                                             'target_locked',
                                             'preempted',
                                             'reachyHome',
                                             'relax'])
        self.rate = rospy.Rate(RATE) # 10hz
        self._mutex = Lock()
        self.mo = mo
        self.reachyRelax = rospy.ServiceProxy('relax', Relax)

    def execute(self, userdata):
        rospy.loginfo("Executing EXTEND State")
        while True:

            if self.preempt_requested():
                rospy.loginfo("preempt triggered")
                relaxReq = RelaxRequest()
                relaxReq.side = "left"
                self.reachyRelax(relaxReq)

            self.rate.sleep()

            local_approach_pose = copy.deepcopy(self.mo.grasp_pose)
            # rospy.loginfo(local_approach_pose)

            #TODO: UNCOMMENT AFTER EXPERIMENT
            # result = self.mo.goToPose(local_approach_pose)
            rospy.loginfo("SIMULATING EXTEND")
            rospy.loginfo(f"-----EXTENDING {self.mo.apriltag_first_elem} -----")
            rospy.sleep(7)
            #TODO: DELETE BELOW. ONLY FOR SIMULATION
            result = 3

            if result == 0: # no plan
                rospy.loginfo("couldnt find a plan. attempting to find a plan again...")
                rospy.loginfo("------------------------------")
                continue
            elif result == 1: # target changed
                rospy.loginfo("target changed. attempting to find a plan again...")
                rospy.loginfo("------------------------------")
                continue
            elif result == 2: # error
                rospy.loginfo("error in actuators")
                rospy.loginfo("------------------------------")
                return "relax"
            elif result == 3: # successful
                rospy.loginfo("successfully moved to goal. changing to extend" )
                rospy.loginfo("------------------------------")
                return 'target_locked'
            else: # successfully moved but not within tolerance #TODO: This is a "minor" issue that requires calibration
                rospy.loginfo("successfully moved to goal. changing to extend" )
                rospy.loginfo("------------------------------")
                continue
        
            

# define state Grasp. This is actually moving the gripper to hold the cube
class Grasp(smach.State):
    def __init__(self, mo):
        smach.State.__init__(self, outcomes=['reachyHome',
                                             'preempted'])
        self.rate = rospy.Rate(RATE) # 10hz
        self._mutex = Lock()
        self.mo = mo

    def execute(self, userdata):
        rospy.loginfo("Executing GRASP State")
        while True:

            if self.preempt_requested():
                rospy.loginfo("preempt triggered")
                return 'preempted'
            
            rospy.loginfo(f"Grasping {self.mo.apriltag_first_elem} object...")

            # TODO: service to grasp the obejct
            rospy.sleep(7) # simulating grasping

            rospy.loginfo(f"Grasped {self.mo.apriltag_first_elem} object")

            # set this flag to true so that reachy will return the object to home and not approach in Approach state
            self.mo.return_object = True

            self.rate.sleep()
            rospy.loginfo("------------------------------")
            return 'reachyHome'

# define state Grasp. This is actually moving the gripper to hold the cube
class MoveToAprilTagHome(smach.State):
    def __init__(self, mo):
        smach.State.__init__(self, outcomes=['release',
                                             'preempted'])
        self.rate = rospy.Rate(RATE) # 10hz
        self._mutex = Lock()
        self.mo = mo


    def execute(self, userdata):
        rospy.loginfo("Executing MOVE APRILTAG HOME State")

        self.current_apriltagHome = self.mo.apriltag_data[self.mo.apriltag_first_elem]
        self.apriltagHomePose = Pose()
        self.apriltagHomePose.position.x = self.current_apriltagHome['position']['x']
        self.apriltagHomePose.position.y = self.current_apriltagHome['position']['y']
        self.apriltagHomePose.position.z = self.current_apriltagHome['position']['z']
        q = quaternion_from_euler(0.0, -math.pi*0.5, 0.0)
        self.apriltagHomePose.orientation.x = q[0]
        self.apriltagHomePose.orientation.y = q[1]
        self.apriltagHomePose.orientation.z = q[2]
        self.apriltagHomePose.orientation.w = q[3]

        while True:
            
            if self.preempt_requested():
                rospy.loginfo("preempt triggered")
                return 'preempted'

            #TODO: reads the first element from the self.apriltag_home_list (X)
            #TODO: perform the move. if successfully, pop that off the list. 
            #TODO: in the reachyhome, if the list is not empty,it'll continue. If not, it'll relax

            #TODO: UNCOMMENT AFTER EXPERIMENT
            self.mo.approach_pose = self.apriltagHomePose

            #TODO: Move the arm to the ready pose and send it to movegroup
            rospy.loginfo(f"MOVING THE ARM TO {self.mo.apriltag_first_elem} HOME POSITION...")
            
            #TODO: UNCOMMENT AFTER EXPERIMENT
            # result = self.mo.goToPose(self.apriltagHomePose)
            rospy.loginfo("SIMULATING MOVE APRILTAG HOME")
            rospy.sleep(7)
            #TODO : DELETE BELOW. ONLY FOR SIMULATION
            result = 3

            rospy.loginfo(f"MOVED THE ARM TO {self.mo.apriltag_first_elem} HOME POSITION")
            

            # result = self.mo.goToPose(self.readyPose)


            self.rate.sleep()

            if result == 3:
                # if self.mo.return_object:
                #     self.mo.return_object = False
                #     return 'apriltagHome'
                # else: 
                #     return 'approach'
                rospy.loginfo("------------------------------")
                return 'release'

            
            self.rate.sleep()

# define state Grasp. This is actually moving the gripper to hold the cube
class MoveToReachyHome(smach.State):
    def __init__(self, mo):
        smach.State.__init__(self, outcomes=['approach',
                                             'rest',
                                             'preempted',
                                             'apriltagHome'])
        self.rate = rospy.Rate(RATE) # 10hz
        self._mutex = Lock()
        self.flag = True
        self.pose = None
        self.mo = mo
        
        self.readyPose = Pose()
        self.readyPose.position.x = 0.1
        self.readyPose.position.y = 0.2 
        self.readyPose.position.z = 0.75
        q = quaternion_from_euler(0.0, -math.pi*0.5, 0.0)
        self.readyPose.orientation.x = q[0]
        self.readyPose.orientation.y = q[1]
        self.readyPose.orientation.z = q[2]
        self.readyPose.orientation.w = q[3]

    def execute(self, userdata):
        rospy.loginfo("Executing Move Reachy Home State")
        
        while True:
            
            if self.preempt_requested():
                rospy.loginfo("preempt triggered")
                return 'preempted'
            
            self.mo.approach_pose = self.readyPose
            #TODO: Move the arm to the ready pose and send it to movegroup
            rospy.loginfo("MOVING THE ARM TO HOME POSITION...")


            #TODO: UNCOMMENT AFTER EXPERIMENT
            # result = self.mo.goToPose(self.readyPose)
            rospy.loginfo("SIMULATING MOVE REACHY HOME")
            rospy.sleep(7)
            result = 3

            rospy.loginfo("MOVED THE ARM TO HOME POSITION")
            
            self.rate.sleep()


            rospy.loginfo(f'There are {len(self.mo.apriltag_home_list)} many objects left')
            if result == 3:
                if len(self.mo.apriltag_home_list) == 0: #finished everything
                    rospy.loginfo("finished everything")
                    rospy.loginfo("------------------------------")
                    return 'rest'
                if self.mo.return_object:
                    self.mo.return_object = False
                    rospy.loginfo("------------------------------")
                    return 'apriltagHome'
                else: 
                    rospy.loginfo("------------------------------")
                    return 'approach'



# define state Rest
class Rest(smach.State):
    def __init__(self,mo):
        smach.State.__init__(self, outcomes=['preempted',
                                             'idle'])
        self._mutex = Lock()
        self.mo = mo
        self.rate = rospy.Rate(RATE) # 10hz
        # rospy.Subscriber('state_transition', String, self._state_transition)

        self.restPose = Pose()
        self.restPose.position.x = 0.0
        self.restPose.position.y = 0.2 
        self.restPose.position.z = 0.354
        q = quaternion_from_euler(0.0, 0.0, 0.0)
        self.restPose.orientation.x = q[0]
        self.restPose.orientation.y = q[1]
        self.restPose.orientation.z = q[2]
        self.restPose.orientation.w = q[3]
        
    def _state_transition(self,msg):
        rospy.loginfo("IN REST CALLBACK, state transition")
        with self._mutex:
            self.data = msg.data

    def execute(self, userdata):
        rospy.loginfo("Executing REST State")
        while True:

            #TODO: Fix this bug. Doesn't rest and errors out
            rospy.wait_for_service('relax')
            reachyRelax = rospy.ServiceProxy('relax', Relax)

            relaxReq = RelaxRequest()
            relaxReq.side = "left"
            result = reachyRelax(relaxReq)

            if self.preempt_requested():
                rospy.loginfo("preempt triggered")
                return 'preempted'

            self.rate.sleep()

# define state Grasp. This is actually moving the gripper to hold the cube
class Release(smach.State):
    def __init__(self,mo):
        smach.State.__init__(self, outcomes=['reachyHome',
                                             'preempted'])
        self.rate = rospy.Rate(RATE) # 10hz
        self._mutex = Lock()
        self.mo = mo

    def execute(self, userdata):
        rospy.loginfo("Executing RELEASE State")
        while True:
            
            if self.preempt_requested():
                    rospy.loginfo("preempt triggered")
                    return 'preempted'

            rospy.loginfo(f"Releasing {self.mo.apriltag_first_elem} object...")

            # TODO: service to release the object
            rospy.sleep(7) # simulating releasing

            rospy.loginfo(f"Released {self.mo.apriltag_first_elem} object")

            # set this flag to true so that reachy will return the object to home and not approach in Approach state
            # self.mo.return_object = True

            # pop out the apriltag list
            # rospy.loginfo(self.mo.apriltag_home_list)
            # rospy.loginfo("popping")
            self.mo.apriltag_home_list.pop(0)
            # self.mo.apriltag_first_elem = self.mo.apriltag_home_list[0]
            rospy.loginfo(f'Remaining objects: {self.mo.apriltag_home_list}')
            
            # print(f'remaining apriltag {self.mo.apriltag_home_list.pop(0)}')

            self.rate.sleep()
            rospy.loginfo("------------------------------")
            return 'reachyHome'



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
    move_interface_object = move_interface.MoveGroupPythonInterfaceTutorial("left")
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
                                            'preempted': 'exit',
                                            'apriltagHome' : 'MOVETOAPRILTAGHOME'})

        smach.StateMachine.add('APPROACH', Approach(move_interface_object), 
                               transitions={'approach':'APPROACH',
                                            'target_locked':'EXTEND',
                                            'reachyHome':'MOVETOREACHYHOME',
                                            'preempted': 'exit',
                                            'relax':'REST'})
                                # remapping={'approach_in':'move_interface_object',
                                #             'approach_in':'move_interface_object'})
                                            
        smach.StateMachine.add('EXTEND', Extend(move_interface_object), 
                               transitions={'approach':'APPROACH',
                                            'target_locked':'GRASP',
                                            'reachyHome':'MOVETOREACHYHOME',
                                            'preempted': 'exit',
                                            'relax':'REST'})
                                # remapping={'extend_in':'move_interface_object',
                                #             'extend_out':'move_interface_object'})
        smach.StateMachine.add('GRASP', Grasp(move_interface_object), 
                               transitions={'reachyHome' : 'MOVETOREACHYHOME',
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