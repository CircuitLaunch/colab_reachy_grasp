#!/usr/bin/env python3

import rospy
from rospy.core import rospyinfo
from DXLProxy import *
from rospy.impl.transport import Transport
import smach
from tf.transformations import *
import math
from threading import Lock, Thread, local
from apriltag_ros.msg import AprilTagDetectionArray
from geometry_msgs.msg import PoseStamped, Pose
from std_msgs.msg import String
from colab_reachy_control.srv import SetGripperPos, WriteRegisters,WriteRegistersRequest, ReadRegisters,SetGripperPosRequest, Relax, RelaxRequest, Zero, ZeroRequest, RestPoseRequest,RestPose
import time
import copy
import move_interface
import geometry_msgs.msg
from colab_reachy_calibrate.srv import TransformPose, TransformPoseRequest, TransformPoseResponse

SIM = False
RATE = 10
# define state Idle. 
class Idle(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['reachyHome',
                                             'preempted',
                                             'grasp',
                                             'rest',
                                             'release'])

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
        gripper = Gripper('left')
        gripper.openGripper()

        while True:
            # rospy.loginfo(self.data)
            with self._mutex:
                if self.data == "start":
                    print(self.data)
                    self.data = None
                    rospy.loginfo("------------------------------")
                    return 'reachyHome'
                elif self.data == "grasp":
                    return 'grasp'
                elif self.data == 'rest':
                    return 'rest'
                elif self.data == 'release':
                    return 'release'
            if self.preempt_requested():
                rospy.loginfo("preempt triggered")
                return 'preempted'
                
            self.rate.sleep()

def calcYaw(x, y):
    '''
    This method returns the angle in radians in which reachy should rotate its gripper give the pose x and y
    '''
    minx = 0.2
    maxx = 0.5 
    miny = 0.0
    maxy = 0.3
    nx = (maxx - x) / (maxx - minx)
    width = maxy - miny
    ny = (width - abs(y)) / width
    return math.pi * (0.16666666666667 * nx + 0.16666666666667 * ny)

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
        self.compensator = rospy.ServiceProxy('compensate', TransformPose)
        self.setGripperPos = rospy.ServiceProxy('set_gripper_pos', SetGripperPos)

    

    def execute(self, userdata):
        rospy.loginfo("Executing APPROACH State")
        
        local_approach_pose = geometry_msgs.msg.Pose()

        while True:

            if self.preempt_requested():
                rospy.loginfo("preempt triggered")
                relaxReq = RelaxRequest()
                relaxReq.side = "left"
                self.reachyRelax(relaxReq)
                return 'relax'

            # self.rate.sleep()
            rospy.loginfo(f"approach state get cube :{self.mo.grasping_mode}")
            
            # grasping mode
            if self.mo.grasping_mode:
                # Update the approach and grasp pose:
                self.mo.lookup_tf()

                local_approach_pose = copy.deepcopy(self.mo.grasp_approach_pose)
                rospy.loginfo(f"GRASP APPROACH POSE for {self.mo.apriltag_first_elem} object: {local_approach_pose}")

            # releasing mode
            else:
                local_approach_pose = copy.deepcopy(self.mo.release_approach_pose)
                rospy.loginfo(f"RELEASE APPROACH POSE for {self.mo.apriltag_first_elem} object: {local_approach_pose}")


            # Error compensation
            transformReq = TransformPoseRequest()
            transformReq.side = 'left'
            transformReq.in_pose = local_approach_pose

            # resp_pose is the error compensated pose
            resp_pose = self.compensator(transformReq)

            if resp_pose.in_bounds == False:
                rospy.loginfo(f"Requested approach pose: {local_approach_pose} is outside the workspace boundary.")
                return 'relax'

            x = resp_pose.out_pose.position.x
            y = resp_pose.out_pose.position.y
            yaw = calcYaw(x,y)
            q = quaternion_from_euler(0.0, -math.pi*0.5, -yaw)
            resp_pose.out_pose.orientation.x = q[0]
            resp_pose.out_pose.orientation.y = q[1]
            resp_pose.out_pose.orientation.z = q[2]
            resp_pose.out_pose.orientation.w = q[3]

            if not SIM:
                result = self.mo.goToPose(resp_pose.out_pose)
        
            if SIM:
                rospy.loginfo(f"SIMULATING APPROACHING {self.mo.apriltag_first_elem}...")
                rospy.loginfo(f"ORIGINAL APPROACH POSE: {local_approach_pose}, COMPENSATED APPROACH POSE: {resp_pose.out_pose}")
                rospy.sleep(5)
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

                if self.mo.grasping_mode:
                    if self.mo.object_in_hand:
                        rospy.loginfo("Have cube in hand. Moving to Reachy Home." )
                        rospy.loginfo("------------------------------")
                        return 'reachyHome'
                    else:
                        rospy.loginfo("Don't have cube in hand. Moving to extend (GRASPING CUBE)" )
                        rospy.loginfo("------------------------------")
                        return 'target_locked'
                else:
                    if self.mo.object_in_hand:
                        rospy.loginfo("Have cube in hand. Moving to extend (RELEASING CUBE)" )
                        rospy.loginfo("------------------------------")                        
                        return 'target_locked'
                    else:
                        rospy.loginfo("Don't have cube in hand. Moving to Reachy Home" )
                        rospy.loginfo("------------------------------")
                        return 'reachyHome'

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
                                             'release',
                                             'relax'])
        self.rate = rospy.Rate(RATE) # 10hz
        self._mutex = Lock()
        self.mo = mo
        self.reachyRelax = rospy.ServiceProxy('relax', Relax)
        self.compensator = rospy.ServiceProxy('compensate', TransformPose)

    def execute(self, userdata):
        rospy.loginfo("Executing EXTEND State")

        local_extend_pose = geometry_msgs.msg.Pose()

        while True:

            if self.preempt_requested():
                rospy.loginfo("preempt triggered")
                relaxReq = RelaxRequest()
                relaxReq.side = "left"
                self.reachyRelax(relaxReq)
                return 'relax'

            if self.mo.grasping_mode:
                local_extend_pose = copy.deepcopy(self.mo.grasp_pose)
                rospy.loginfo(f"RELEASE EXTEND POSE for {self.mo.apriltag_first_elem} object: {local_extend_pose}")
            else:
                local_extend_pose = copy.deepcopy(self.mo.release_pose)
                rospy.loginfo(f"RELEASE EXTEND POSE for {self.mo.apriltag_first_elem} object: {local_extend_pose}")
           
            #Error compensation
            transformReq = TransformPoseRequest()
            transformReq.side = 'left'
            transformReq.in_pose = local_extend_pose
            
            # resp_pose is the error compensate pose
            resp_pose = self.compensator(transformReq)

            if resp_pose.in_bounds == False:
                rospy.loginfo(f"Requested extend pose: {local_extend_pose} is outside the workspace boundary.")
                return 'relax'

            x = resp_pose.out_pose.position.x
            y = resp_pose.out_pose.position.y
            yaw = calcYaw(x,y)
            q = quaternion_from_euler(0, -math.pi*0.5, -yaw)
            resp_pose.out_pose.orientation.x = q[0]
            resp_pose.out_pose.orientation.y = q[1]
            resp_pose.out_pose.orientation.z = q[2]
            resp_pose.out_pose.orientation.w = q[3]
            
            if not SIM:
                result = self.mo.goToPose(resp_pose.out_pose)

            if SIM:
                rospy.loginfo(f"SIMULATING EXTEND {self.mo.apriltag_first_elem}...")
                rospy.loginfo(f"ORIGINAL EXTEND POSE: {local_extend_pose}, COMPENSATED EXTEND POSE: {resp_pose.out_pose}")
                rospy.sleep(5)
                result = 3

            if result == 0: # no plan
                rospy.loginfo("couldnt find a plan. attempting to find a plan again...")
                rospy.loginfo("------------------------------")
                return 'approach'
                
            elif result == 1: # target changed
                rospy.loginfo("target changed. attempting to find a plan again...")
                rospy.loginfo("------------------------------")
                # self.mo.lookup = True
                return 'approach'

            elif result == 2: # error
                rospy.loginfo("error in actuators")
                rospy.loginfo("------------------------------")
                return "relax"
                
            elif result == 3: # successful
                
                if self.mo.grasping_mode:
                    rospy.loginfo("successfully moved to goal. changing to extend" )
                    rospy.loginfo("------------------------------")
                    return 'target_locked'
                else:
                    rospy.loginfo("successfully moved to goal. changing to Release" )
                    rospy.loginfo("------------------------------")
                    return 'release'
            else: # successfully moved but not within tolerance #TODO: This is a "minor" issue that requires calibration
                rospy.loginfo("successfully moved to goal. changing to extend" )
                rospy.loginfo("------------------------------")
                # self.mo.lookup = True
                return 'approach'
            


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
        
        self.readyPose = Pose()
        self.readyPose.position.x = 0.05
        self.readyPose.position.y = 0.2 
        self.readyPose.position.z = 0.75
        q = quaternion_from_euler(0.0, -math.pi*0.5, 0.0)
        self.readyPose.orientation.x = q[0]
        self.readyPose.orientation.y = q[1]
        self.readyPose.orientation.z = q[2]
        self.readyPose.orientation.w = q[3]

    def execute(self, userdata):
        rospy.loginfo("Executing MOVE REACHY HOME State")
        
        while True:
            
            if self.preempt_requested():
                rospy.loginfo("preempt triggered")
                return 'preempted'
                        
            self.mo.approach_pose = self.readyPose

            if not SIM:
                result = self.mo.goToPose(self.readyPose)
            if SIM:
                rospy.loginfo("SIMULATING MOVE REACHY HOME state")
                result = 3
                rospy.sleep(7)

            self.rate.sleep()

            rospy.loginfo(f'There are {len(self.mo.apriltag_home_list)} many objects left')
            if result == 3:
                if len(self.mo.apriltag_home_list) == 0: #finished everything
                    rospy.loginfo("Finished moving all objects with apriltags")
                    rospy.loginfo("------------------------------")
                    return 'rest'
                else: 
                    rospy.loginfo(f"grasping_mode state in reachy home: {self.mo.grasping_mode}")
                    if self.mo.grasping_mode:
                        self.mo.grasping_mode= False
                    else:
                        self.mo.grasping_mode = True
                    rospy.loginfo("------------------------------")
                    return 'approach'


# define state Rest
class Rest(smach.State):
    def __init__(self,mo):
        smach.State.__init__(self, outcomes=['preempted',
                                             'idle',
                                             'exit'])
        self._mutex = Lock()
        self.mo = mo
        self.rate = rospy.Rate(RATE) # 10hz
        self.zero = rospy.ServiceProxy('zero', Zero)
        self.reachyRelax = rospy.ServiceProxy('relax', Relax)
        self.restPose = rospy.ServiceProxy('rest_pose', RestPose)

    def _state_transition(self,msg):
        rospy.loginfo("IN REST CALLBACK, state transition")
        with self._mutex:
            self.data = msg.data

    def execute(self, userdata):
        rospy.loginfo("Executing REST State")
        while True:

            if self.preempt_requested():
                rospy.loginfo("preempt triggered")
                return 'preempted'

            # close gripper
            gripper = Gripper('left')
            gripper.closeGripper()

            # move reachy left arm to rest pose
            self.goToRestPose('left')

            # Turn all torque off
            relaxReq = RelaxRequest()
            relaxReq.side = 'left'
            self.reachyRelax(relaxReq)
            rospy.loginfo('relaxing')

            self.rate.sleep()
            
            return 'idle'

    def goToRestPose(self, side):
        restPoseReq = RestPoseRequest()
        restPoseReq.side = side
        restPoseReq.speed = 0.05 
        self.restPose(restPoseReq)


# define state Grasp. This is actually moving the gripper to hold the cube
class Grasp(smach.State):
    def __init__(self, mo):
        smach.State.__init__(self, outcomes=['reachyHome',
                                             'preempted',
                                             'approach',
                                             'idle'])
        self.rate = rospy.Rate(RATE) # 10hz
        self._mutex = Lock()
        self.mo = mo
        self.setGripperPos = rospy.ServiceProxy('set_gripper_pos', SetGripperPos)
        self.dxl_proxy = DXLProxy()

        self.dxl_proxy.writeRegisters([27],[RAM_MOVING_SPEED],[0.2])

    def execute(self, userdata):
        rospy.loginfo("Executing GRASP State")
        while True:

            if self.preempt_requested():
                rospy.loginfo("preempt triggered")
                return 'idle'
            
            rospy.loginfo(f"Grasping {self.mo.apriltag_first_elem} object...")

            gripper = Gripper('left')
            gripper.closeGripper()

            if SIM:
                rospy.sleep(7) # simulating grasping

            rospy.loginfo(f"Grasped {self.mo.apriltag_first_elem} object")

            # set this flag to true so that reachy will return the object to home and not approach in Approach state
            self.mo.object_in_hand = True
            
            self.rate.sleep()
            rospy.loginfo("------------------------------")
            return 'approach'

# define state Grasp. This is actually moving the gripper to hold the cube
class Release(smach.State):
    def __init__(self,mo):
        smach.State.__init__(self, outcomes=['reachyHome',
                                             'approach',
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

            gripper = Gripper('left')
            gripper.openGripper()

            if SIM:
                rospy.sleep(7) # simulating releasing

            rospy.loginfo(f"Released {self.mo.apriltag_first_elem} object")

            self.mo.object_in_hand = False

            # Once the cube is released, pop out the current element out from the list
            self.mo.apriltag_home_list.pop(0)
            if len(self.mo.apriltag_home_list) != 0:
                self.mo.apriltag_first_elem = self.mo.apriltag_home_list[0]
            
            rospy.loginfo(f'Remaining objects: {self.mo.apriltag_home_list}')
            rospy.loginfo(f"next apriltag: {self.mo.apriltag_first_elem}")
            self.mo.object_in_hand = False
            self.rate.sleep()
            rospy.loginfo("------------------------------")
            return 'approach'

class Gripper():

    def __init__(self,side):
        self.setGripperPos = rospy.ServiceProxy('set_gripper_pos', SetGripperPos)
        self.side = side

    def openGripper(self):
        setGripperPosReq = SetGripperPosRequest()
        setGripperPosReq.side = self.side
        setGripperPosReq.angle = self.deg_to_rad(190)
        rospy.loginfo(f"angle for open gripper: {setGripperPosReq.angle}")
        self.setGripperPos(setGripperPosReq)

    def closeGripper(self):
        setGripperPosReq = SetGripperPosRequest()
        setGripperPosReq.side = self.side
        setGripperPosReq.angle = self.deg_to_rad(144)
        rospy.loginfo(f"angle for close gripper: {setGripperPosReq.angle}")
        self.setGripperPos(setGripperPosReq)

    def deg_to_rad(self, dynamixel_ang):
        '''
        dynamixel degree to reachy compatible rad
        '''
        dynamixel_step = dynamixel_ang/0.29
        reachy_step = dynamixel_step - 512
        reachy_deg = reachy_step * 0.29
        reachy_rad = math.radians(reachy_deg)
        return reachy_rad


# main
def main():
    rospy.init_node('smach_example_state_machine')

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['exit'])

    #TODO: create an isntance of move interface and pass it among approach, extend and grasp.
    # This is important because this object contains all the attributes of the system and must be shared 
    move_interface_object = move_interface.MoveGroupPythonInterfaceTutorial("left")

    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('IDLE', Idle(), 
                               transitions={'reachyHome':'MOVETOREACHYHOME', 
                                            'preempted': 'exit',
                                            'grasp': 'GRASP',
                                            'release': 'RELEASE',
                                            'rest': 'REST'})

        smach.StateMachine.add('MOVETOREACHYHOME', MoveToReachyHome(move_interface_object), 
                               transitions={'approach':'APPROACH',
                                            'rest':'REST',
                                            'preempted': 'exit'})

        smach.StateMachine.add('APPROACH', Approach(move_interface_object), 
                               transitions={'approach':'APPROACH',
                                            'target_locked':'EXTEND',
                                            'reachyHome':'MOVETOREACHYHOME',
                                            'preempted': 'exit',
                                            'relax':'REST'})
                                            
        smach.StateMachine.add('EXTEND', Extend(move_interface_object), 
                               transitions={'approach':'APPROACH',
                                            'target_locked':'GRASP',
                                            'reachyHome':'MOVETOREACHYHOME',
                                            'preempted': 'exit',
                                            'release':'RELEASE',
                                            'relax':'REST'})

        smach.StateMachine.add('GRASP', Grasp(move_interface_object), 
                               transitions={'reachyHome' : 'MOVETOREACHYHOME',
                                            'preempted': 'exit',
                                            'approach': 'APPROACH',
                                            'idle': 'IDLE'})

        smach.StateMachine.add('REST', Rest(move_interface_object), 
                               transitions={'idle':'IDLE', 
                                            'preempted': 'exit',
                                            'exit': 'exit'})

        smach.StateMachine.add('RELEASE', Release(move_interface_object), 
                                transitions={'reachyHome':'MOVETOREACHYHOME', 
                                            'approach' : 'APPROACH',
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