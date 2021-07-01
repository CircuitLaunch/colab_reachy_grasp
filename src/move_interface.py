#!/usr/bin/env python3
import math
from os import RTLD_GLOBAL
import time
from rospy.rostime import get_rostime_cond
import tf2_ros
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
from moveit_msgs.msg import MoveItErrorCodes
import geometry_msgs.msg
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from apriltag_ros.msg import AprilTagDetection, AprilTagDetectionArray
from math import pi, tau, dist, fabs, cos
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
from threading import Lock, Thread
import pdb
## END_SUB_TUTORIAL


def all_close(goal, actual, tolerance):
    """
    Convenience method for testing if the values in two lists are within a tolerance of each other.
    For Pose and PoseStamped inputs, the angle between the two quaternions is compared (the angle
    between the identical orientations q and -q is calculated correctly).
    @param: goal       A list of floats, a Pose or a PoseStamped
    @param: actual     A list of floats, a Pose or a PoseStamped
    @param: tolerance  A float
    @returns: bool
    """
    if type(goal) is list:
        for index in range(len(goal)):
            if abs(actual[index] - goal[index]) > tolerance:
                return False

    elif type(goal) is geometry_msgs.msg.PoseStamped:
        return all_close(goal.pose, actual.pose, tolerance)

    elif type(goal) is geometry_msgs.msg.Pose:
        x0, y0, z0, qx0, qy0, qz0, qw0 = pose_to_list(actual)
        x1, y1, z1, qx1, qy1, qz1, qw1 = pose_to_list(goal)
        # Euclidean distance
        d = dist((x1, y1, z1), (x0, y0, z0))
        # phi = angle between orientations
        cos_phi_half = fabs(qx0*qx1 + qy0*qy1 + qz0*qz1 + qw0*qw1)
        return d <= tolerance and cos_phi_half >= cos(tolerance / 2.0)

    return True


class MoveGroupPythonInterfaceTutorial(object):
    """MoveGroupPythonInterfaceTutorial"""
    def __init__(self):

        super(MoveGroupPythonInterfaceTutorial, self).__init__()

        ## First initialize `moveit_commander`_ and a `rospy`_ node:
        moveit_commander.roscpp_initialize(sys.argv)




        # UNCOMMENT THIS PART IF YOU WANT TO RUN THE MOVE_INTERFACE BY ITSELF
        # rospy.init_node('move_group_python_interface_tutorial', anonymous=True)








        ## Instantiate a `RobotCommander`_ object. Provides information such as the robot's
        ## kinematic model and the robot's current joint states
        robot = moveit_commander.RobotCommander()

        ## Instantiate a `PlanningSceneInterface`_ object.  This provides a remote interface
        ## for getting, setting, and updating the robot's internal understanding of the
        ## surrounding world:
        scene = moveit_commander.PlanningSceneInterface()

        ## Instantiate a `MoveGroupCommander`_ object.  This object is an interface
        ## to a planning group (group of joints).  In this tutorial the group is the primary
        ## arm joints in the Panda robot, so we set the group's name to "panda_arm".
        ## If you are using a different robot, change this value to the name of your robot
        ## arm planning group.
        ## This interface can be used to plan and execute motions:
        group_name = "left_arm"
        move_group = moveit_commander.MoveGroupCommander(group_name)

        ## Create a `DisplayTrajectory`_ ROS publisher which is used to display
        ## trajectories in Rviz:
        display_trajectory_publisher = rospy.Publisher('/display_planned_path',
                                                       moveit_msgs.msg.DisplayTrajectory,
                                                       queue_size=20)
                                                      # NOTE: Ed's code didn't have /move_group


        ## Getting Basic Information
        ## ^^^^^^^^^^^^^^^^^^^^^^^^^
        # We can get the name of the reference frame for this robot:
        planning_frame = move_group.get_planning_frame()
        print("============ Planning frame: %s" % planning_frame)

        # We can also print the name of the end-effector link for this group:
        eef_link = move_group.get_end_effector_link()
        print("============ End effector link: %s" % eef_link)

        # We can get a list of all the groups in the robot:
        group_names = robot.get_group_names()
        print("============ Available Planning Groups:", robot.get_group_names())

        # Sometimes for debugging it is useful to print the entire state of the
        # robot:
        print("============ Printing robot state")
        print(robot.get_current_state())
        print("")
        ## END_SUB_TUTORIAL

        # for i in range(len(move_group.get_current_joint_values())):
        #   print(move_group.get_current_joint_values()[i])

        # Misc variables
        self.box_name = ''
        self.robot = robot
        self.scene = scene
        self.move_group = move_group
        self.display_trajectory_publisher = display_trajectory_publisher
        self.planning_frame = planning_frame
        self.eef_link = eef_link
        self.group_names = group_names
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)

        self.spin_rate = rospy.Rate(30)

        self.grasping_state_init()
        # self.grasping_thread = Thread(target=self.grasping_state_loop)
        # self.grasping_thread.start()

        # local_approach_pose = self.current_pose()

    # def execute_to_goal(self, msg):
    #   appraoch_to_goal(self, msg)
    #   go_to_pose_goal(self, msg)

    def go_to_pose_goal(self, msg):

        #initialize cube position and looping rate
        rate = rospy.Rate(5.0)
        # rate2 = rospy.Rate(1)
        # cube_x = 0
        # cube_y = 0
        # cube_z = 0


        # rate2.sleep()
        #attempts to get the latest transform of the cube from world
        try:
            trans = self.tfBuffer.lookup_transform('pedestal', 'cube2',rospy.Time(0))
            for a in msg.detections:
                if a.id[0] == 5:
                    # rospy.loginfo("Found apriltag")
                    cube_pose = geometry_msgs.msg.Pose()
                    cube_pose.position = trans.transform.translation
                    cube_pose.orientation = trans.transform.rotation
                    self.on_cube_detected(cube_pose)



        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rate.sleep()


       

    def grasping_state_init(self):
        self._min_approach_error = rospy.get_param("MIN_APPROACH_ERROR") # TODO: determine a suitable minimum approach pose error
        self._min_grip_error = rospy.get_param("MIN_GRIP_ERROR") # TODO: determine a suitable minimum grip pose error
        self._max_grip_load = rospy.get_param("MAX_GRIP_LOAD")
        self._approach_x_offset = rospy.get_param("APPROACH_X_OFFSET")
        self._approach_y_offset = rospy.get_param("APPROACH_Y_OFFSET")
        self._approach_z_offset = rospy.get_param("APPROACH_Z_OFFSET")
        self._grasp_x_offset = rospy.get_param("GRASP_X_OFFSET")
        self._grasp_y_offset = rospy.get_param("GRASP_Y_OFFSET")
        self._grasp_z_offset = rospy.get_param("GRASP_Z_OFFSET")
        

        self._settings_mutex = Lock()

        self._grip_load = 0.0
        self._grip_load_mutex = Lock()
        self._abort = False
        self._abort_mutex = Lock()

        self._cube_pose = geometry_msgs.msg.Pose()
        self._cube_pose_mutex = Lock()
        self._approach_pose = geometry_msgs.msg.Pose()
        self._approach_pose_mutex = Lock()
        self._grasp_pose = geometry_msgs.msg.Pose()
        self._grasp_pose_mutex = Lock()
        self._current_pose = geometry_msgs.msg.Pose()   # TODO: do we pull current pose from tf, or does it publish it somewhere?
        self._current_pose_mutex = Lock()

        self.trajectory_in_progress = False

        # TODO: implement some way to trigger grasping_state_loop() in a new thread
        #       from a state machine?!?

    @property
    def min_approach_error(self):
        local_val = 0.0
        with self._settings_mutex:
            local_val = self._min_approach_error
        return local_val

    @min_approach_error.setter
    def min_approach_error(self, val):
        with self._settings_mutex:
            self._min_approach_error = val

    @property
    def min_grip_error(self):
        local_val = 0.0
        with self._settings_mutex:
            local_val = self._min_grip_error
        return local_val

    @min_grip_error.setter
    def min_grip_error(self, val):
        with self._settings_mutex:
            self._min_grip_error = val

    @property
    def approach_x_offset(self):
        local_val = 0
        with self._settings_mutex:
            local_val = self._approach_x_offset
        return local_val

    @approach_x_offset.setter
    def approach_x_offset(self, val):
        with self._settings_mutex:
            self._approach_x_offset = val

    @property
    def approach_y_offset(self):
        local_val = 0
        with self._settings_mutex:
            local_val = self._approach_y_offset
        return local_val

    @approach_y_offset.setter
    def approach_y_offset(self, val):
        with self._settings_mutex:
            self._approach_y_offset = val
    
    @property       
    def approach_z_offset(self):
        local_val = 0
        with self._settings_mutex:
            local_val = self._approach_z_offset
        return local_val

    @approach_z_offset.setter
    def approach_z_offset(self, val):
        with self._settings_mutex:
            self._approach_z_offset = val

    @property
    def grasp_x_offset(self):
        local_val = 0
        with self._settings_mutex:
            local_val = self._grasp_x_offset
        return local_val

    @grasp_x_offset.setter
    def grasp_x_offset(self, val):
        with self._settings_mutex:
            self._grasp_x_offset = val

    @property
    def grasp_y_offset(self):
        local_val = 0
        with self._settings_mutex:
            local_val = self._grasp_y_offset
        return local_val

    @grasp_y_offset.setter
    def grasp_y_offset(self, val):
        with self._settings_mutex:
            self._grasp_y_offset = val

    @property
    def grasp_z_offset(self):
        local_val = 0
        with self._settings_mutex:
            local_val = self._grasp_z_offset
        return local_val

    @grasp_z_offset.setter
    def grasp_z_offset(self, val):
        with self._settings_mutex:
            self._grasp_z_offset = val

    @property
    def max_grip_load(self):
        local_val = 1.0
        with self._settings_mutex:
            local_val = self._max_grip_load
        return local_val

    @max_grip_load.setter
    def max_grip_load(self, val):
        with self._settings_mutex:
            self._max_grip_load = val

    @property
    def grip_load(self):
        local_grip_load = 0.0
        with self._grip_load_mutex:
            local_grip_load = self._grip_load
        return local_grip_load

    @grip_load.setter
    def grip_load(self, val):
        with self._grip_load_mutex:
            self._grip_load = val

    @property
    def abort(self):
        local_abort_flag = False
        with self._abort_mutex:
            local_abort_flag = self._abort
        return local_abort_flag

    @abort.setter
    def abort(self, val):
        with self._abort_mutex:
            self._abort = val

    @property
    def approach_pose(self):
        local_pose = None
        with self._approach_pose_mutex:
            local_pose = self._approach_pose
        return local_pose

    @approach_pose.setter
    def approach_pose(self, val):
        with self._approach_pose_mutex:
            self._approach_pose = val

    @property
    def grasp_pose(self):
        local_pose = None
        with self._grasp_pose_mutex:
            local_pose = self._grasp_pose
        return local_pose

    @grasp_pose.setter
    def grasp_pose(self, val):
        with self._grasp_pose_mutex:
            self._grasp_pose = val

    @property
    def cube_pose(self):
        local_pose = None
        with self._cube_pose_mutex:
            local_pose = self._cube_pose
        return local_pose

    @cube_pose.setter
    def cube_pose(self, val):
        with self._cube_pose_mutex:
            self._cube_pose = val

    @property
    def current_pose(self):
        return self.move_group.get_current_pose().pose
        '''
        local_pose = None
        with self._current_pose_mutex:
            local_pose = self._current_pose
        return local_pose
        '''

    '''
    @current_pose.setter
    def current_pose(self, val):
        with self._current_pose_mutex:
            self._current_pose = val
    '''


    def calc_approach_pose(self, cube_pose):
        best_pose = geometry_msgs.msg.Pose()

        # TODO: Calculate the approach pose given the cube pose pose

        # ADDED BY LIAM
        best_pose.position.x = cube_pose.position.x - self.approach_x_offset
        best_pose.position.y = cube_pose.position.y + self.approach_y_offset
        best_pose.position.z = cube_pose.position.z + self.approach_z_offset

        quat = quaternion_from_euler (0.0, -1.57, 0.0)

        best_pose.orientation.x = quat[0]       
        best_pose.orientation.y = quat[1]
        best_pose.orientation.z = quat[2]
        best_pose.orientation.w = quat[3]

        # ADDED BY LIAM

        return best_pose

    def calc_grasp_pose(self, cube_pose):
        best_pose = geometry_msgs.msg.Pose()

        # TODO: Calculate the grasp pose given the cube pose
        
        # ADDED BY LIAM
        best_pose.position.x = cube_pose.position.x + self.grasp_x_offset
        best_pose.position.y = cube_pose.position.y + self.grasp_y_offset
        best_pose.position.z = cube_pose.position.z + self.grasp_z_offset

        quat = quaternion_from_euler (0.0, -1.57, 0.0)

        best_pose.orientation.x = quat[0]       
        best_pose.orientation.y = quat[1]
        best_pose.orientation.z = quat[2]
        best_pose.orientation.w = quat[3]

        # ADDED BY LIAM

        return best_pose

    def on_cube_detected(self, cube_pose):
        self.approach_pose = self.calc_approach_pose(cube_pose)
        # rospy.loginfo("this is the approach pose")
        # rospy.loginfo(self.approach_pose)
        self.grasp_pose = self.calc_grasp_pose(cube_pose)

    def distance(self, pose1, pose2):
        dx = pose1.position.x - pose2.position.x
        dy = pose1.position.y - pose2.position.y
        dz = pose1.position.z - pose2.position.z
        return math.sqrt(dx*dx + dy*dy + dz*dz)

    def pose_transform(self, frame1, frame2):

        trans = self.tfBuffer.lookup_transform('pedestal', 'cube2',rospy.Time(0))
        rospy.loginfo("IN POSE TF")
        rospy.loginfo(trans)
        return trans
        # for a in msg.detections:
        #     if a.id[0] == 5:
        #         # rospy.loginfo("Found apriltag")
        #         cube_pose = geometry_msgs.msg.Pose()
        #         cube_pose.position = trans.transform.translation
        #         cube_pose.orientation = trans.transform.rotation
        #         self.on_cube_detected(cube_pose)



    def go_to_pose(self, pose):

        self.move_group.set_pose_target(pose)
        plan = self.move_group.plan()
        
        if plan[0]:   
            rospy.loginfo("FOUND PLAN") 
            self.move_group.execute(plan[1], wait=False)
            self.trajectory_in_progress = True
            return True
        else:
            self.trajectory_in_progress = False
            rospy.loginfo("no plan")
            # local_approach_pose = copy.deepcopy(self.approach_pose)
            return False


    def abort_trajectory(self):
        self.move_group.stop()
        self.move_group.clear_pose_targets()
        rospy.loginfo("aborted the trj")
        self.trajectory_in_progress = False

    def tighten_grip(self):
        pass

    def min_grip_load(self):
        return 0.2

    def grasping_state_loop(self):
        self.abort = False

        # Do move to approach pose, move to grasp pose and grasp
        while(True):

            # pdb.set_trace()
            self.spin_rate.sleep()
            rospy.loginfo("before abort in grasping")
            if self.abort:
                return False
            rospy.loginfo("between abort and shutdown")
            if rospy.is_shutdown():
                return False
            # Do move to approach pose and move to grasp pose
            while(True):
                rospy.loginfo("in grasping")
                self.spin_rate.sleep()
                if self.abort or rospy.is_shutdown():
                    return False
                approach_pose_attained = False
                grasp_pose_attained = False
                # Do move to approach pose
                # rospy.loginfo("this is the self arpproach pose")
                # rospy.loginfo(self.approach_pose)
                # local_approach_pose = copy.deepcopy(self.approach_pose)

                local_approach_pose = geometry_msgs.msg.Pose()

                # TODO: Calculate the grasp pose given the cube pose
                
                # ADDED BY LIAM
                local_approach_pose.position.x = self.approach_pose.position.x 
                local_approach_pose.position.y = self.approach_pose.position.y 
                local_approach_pose.position.z = self.approach_pose.position.z

                local_approach_pose.orientation.x = self.approach_pose.orientation.x       
                local_approach_pose.orientation.y = self.approach_pose.orientation.y   
                local_approach_pose.orientation.z = self.approach_pose.orientation.z   
                local_approach_pose.orientation.w = self.approach_pose.orientation.w   




                rospy.loginfo("THIS IS THE local_approach_pose: " )
                # rospy.loginfo(local_approach_pose)
                while(True):
                    self.spin_rate.sleep()
                    if self.abort or rospy.is_shutdown():
                        return False
                    ''' # THESE ARE DETERMINED ASYNCHRONOUSLY EVERYTIME THE TAG IS DETECTED
                        # Determine grasp pose
                        # Determine current pose
                    '''
                    # If no trajectory being executed
                    if not self.trajectory_in_progress:
                        # trigger trajectory
                        if not self.go_to_pose(local_approach_pose):
                            local_approach_pose = copy.deepcopy(self.approach_pose)
                        #     rospy.loginfo("inside if")
                        #     rospy.loginfo(local_approach_pose)
                    
                        # rospy.loginfo("outside if")
                        # rospy.loginfo(local_approach_pose)
                    
                    # Else
                    else:
                        # If approach pose has changed
                        if self.distance(self.approach_pose, local_approach_pose) > self.min_approach_error:
                            rospy.loginfo("cube moving")
                            # abort current trajectory
                            self.abort_trajectory()
                            # trigger new trajectory
                            local_approach_pose = copy.deepcopy(self.approach_pose)
                            self.go_to_pose(local_approach_pose)
                    # Repeat move to approach pose while distance between current pose and approach pose > threshold and not abort
                    rospy.loginfo("DISTANCE: ")
                    rospy.loginfo(self.distance(self.current_pose, local_approach_pose))
                    if self.distance(self.current_pose, local_approach_pose) <= self.min_approach_error:
                        approach_pose_attained = True
                        self.trajectory_in_progress = False
                        break

                rospy.loginfo("Completed approach")
                # Do move to grasp pose
                local_grasp_pose = copy.deepcopy(self.grasp_pose)
                
                rospy.loginfo("STARTING GO TO GOAL")
                while(True):
                    self.spin_rate.sleep()
                    if self.abort or rospy.is_shutdown():
                        return False
                    ''' # THESE ARE DETERMINED ASYNCHRONOUSLY EVERYTIME THE TAG IS DETECTED
                        # Determine grasp pose
                        # Determine current pose
                    '''
                    # If no trajectory being executed
                    if not self.trajectory_in_progress:
                        # trigger trajectory
                        # self.go_to_pose(local_grasp_pose)

                        if not self.go_to_pose(local_grasp_pose):
                            local_grasp_pose = copy.deepcopy(self.grasp_pose)
                            rospy.loginfo("created the local_grasp_pose")
                            # rospy.loginfo("inside if")
                            # rospy.loginfo(local_grasp_pose)
                    
                        # rospy.loginfo("outside if")
                        # rospy.loginfo(local_grasp_pose)
                    # Else
                    else:
                        # If grasp pose has changed
                        rospy.loginfo("grasp pose and local grasp pose distance")
                        rospy.loginfo(self.distance(self.grasp_pose, local_grasp_pose))
                        if self.distance(self.grasp_pose, local_grasp_pose) > self.min_grip_error:
                            # abort current trajectory
                            self.abort_trajectory()
                            break
                    # Repeat move to grasp pose while distance between current pose and grasp pose > threshold and not abort
                    if self.distance(self.grasp_pose, local_grasp_pose) <= self.min_grip_error:
                        grasp_pose_attained = True
                        break
                # Repeat move to approach pose and move to grasp pose while distance between current pose and grasp pose > threshold and not abort
                if grasp_pose_attained:
                    break
                
            local_grasp_pose = copy.deepcopy(self.grasp_pose)

            rospy.loginfo("Completed to goal")
            # Do grasp
            local_grasp_flag = False
            self.trajectory_in_progress = False
            while(True):
                self.spin_rate.sleep()
                if self.abort or rospy.is_shutdown():
                    return False
                # Close grip
                self.tighten_grip()
                ''' # THESE ARE DETERMINED ASYNCHRONOUSLY EVERYTIME THE TAG IS DETECTED
                    # Determine grasp pose
                    # Determine current pose
                '''
                # set grasped false
                local_grasp_flag = False
                # If grasp pose has changed
                rospy.loginfo("checking the distance for the grasp")
                rospy.loginfo(self.distance(self.grasp_pose, local_grasp_pose))
                if self.distance(self.grasp_pose, local_grasp_pose) > self.min_grip_error:
                    break
                # set grasped true
                local_grasp_flag = True
                # Repeat while grip load < threshold
                rospy.loginfo("GRASPING")
                if self.grip_load > self.min_grip_load():
                    break
            # Repeat grasp while not (grasped or abort)
            if local_grasp_flag:
                break
        # Return result
        return True


######################################################################################

# Modularizing to use in the state machine


######################################################################################


#we start the node by initializing our move group interface object, and creating a subscriber that listens to the tag detections topic
def main():
    try:
        tutorial = MoveGroupPythonInterfaceTutorial()
        rospy.Subscriber('/tag_detections',
                         AprilTagDetectionArray,
                         tutorial.go_to_pose_goal)
        rospy.spin()

    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        # plan the arm to left_arm rest and then release the arm
        return

if __name__ == '__main__':
    main()
