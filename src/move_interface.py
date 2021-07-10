#!/usr/bin/env python3
import math
from os import RTLD_GLOBAL
import time
from rospy.rostime import get_rostime_cond
import tf2_ros
import tf
import sys
import copy
import rospy
from geometry_msgs.msg import Pose
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
from colab_reachy_control.msg import Telemetry
import json
import rospkg
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
    def __init__(self, side):

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

        # group_name = "left_arm"
        # move_group = moveit_commander.MoveGroupCommander(group_name)
        move_group = moveit_commander.MoveGroupCommander(f'{ side }_arm')

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
        self.flag1 = True
        self.trans = None
        self.hertz = rospy.Rate(30)
        self.grasping_state_init()

        self.restPose = Pose()
        self.restPose.position.x = 0.0
        self.restPose.position.y = 0.2 if side == 'left' else -0.2
        self.restPose.position.z = 0.354
        q = quaternion_from_euler(0.0, 0.0, 0.0)
        self.restPose.orientation.x = q[0]
        self.restPose.orientation.y = q[1]
        self.restPose.orientation.z = q[2]
        self.restPose.orientation.w = q[3]

        self._read_apriltag()
        self.lookup = True
        # self.grasping_thread = Thread(target=self.grasping_state_loop)
        # self.grasping_thread.start()

        # local_grasp_approach_pose = self.current_pose()

    def grasping_state_init(self):
        '''
        This method loads all the parameter values from yaml files to the rosparam
        '''
        self._min_approach_error = rospy.get_param("MIN_APPROACH_ERROR") # TODO: determine a suitable minimum approach pose error
        self._min_grip_error = rospy.get_param("MIN_GRIP_ERROR") # TODO: determine a suitable minimum grip pose error
        self._max_grip_load = rospy.get_param("MAX_GRIP_LOAD")
        self._approach_x_offset = rospy.get_param("APPROACH_X_OFFSET")
        self._approach_y_offset = rospy.get_param("APPROACH_Y_OFFSET")
        self._approach_z_offset = rospy.get_param("APPROACH_Z_OFFSET")
        self._grasp_x_offset = rospy.get_param("GRASP_X_OFFSET")
        self._grasp_y_offset = rospy.get_param("GRASP_Y_OFFSET")
        self._grasp_z_offset = rospy.get_param("GRASP_Z_OFFSET")
        
        self._release_approach_x_offset = rospy.get_param("RELEASE_APPROACH_X_OFFSET")
        self._release_approach_y_offset = rospy.get_param("RELEASE_APPROACH_Y_OFFSET")
        self._release_approach_z_offset = rospy.get_param("RELEASE_APPROACH_Z_OFFSET")
        self._release_x_offset = rospy.get_param("RELEASE_X_OFFSET")
        self._release_y_offset = rospy.get_param("RELEASE_Y_OFFSET")
        self._release_z_offset = rospy.get_param("RELEASE_Z_OFFSET")
        
        telemSub = rospy.Subscriber('dxl_telemetry', Telemetry, self.setTelemetry)
        self._settings_mutex = Lock()

        self._grip_load = 0.0
        self._grip_load_mutex = Lock()
        self._abort = False
        self._abort_mutex = Lock()

        self._cube_pose = geometry_msgs.msg.Pose()
        self._cube_pose_mutex = Lock()
        self._grasp_approach_pose = geometry_msgs.msg.Pose()
        self._grasp_approach_pose_mutex = Lock()
        self._grasp_pose = geometry_msgs.msg.Pose()
        self._grasp_pose_mutex = Lock()
        self._current_pose = geometry_msgs.msg.Pose()   # TODO: do we pull current pose from tf, or does it publish it somewhere?
        self._current_pose_mutex = Lock()

        self.trajectory_in_progress = False

        self.object_in_hand = False
        self.finished_current_job = True
        self.grasping_mode = False

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
    def grasp_approach_pose(self):
        local_pose = None
        with self._grasp_approach_pose_mutex:
            local_pose = self._grasp_approach_pose
        return local_pose

    @grasp_approach_pose.setter
    def grasp_approach_pose(self, val):
        with self._grasp_approach_pose_mutex:
            self._grasp_approach_pose = val

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

    def _read_apriltag(self):
        '''
        sets fields apriltag data that can be used in reachy_fsm
        '''
        # get an instance of RosPack with the default search paths
        rospack = rospkg.RosPack()

        # get the file path for rospy_tutorials
        path = rospack.get_path('colab_reachy_grasp')

        f = open(f'{path}/config/apriltag_home.json',"r")
        self.apriltag_data = json.loads(f.read())
        self.apriltag_home_list = list(self.apriltag_data.keys())
        self.apriltag_first_elem = self.apriltag_home_list[0]

    def lookup_tf(self):
        '''
        This method looks up the home positions of the apriltag which is saved as a json file.
        The home positions of the apriltags are hard coded.
        '''
        rate = rospy.Rate(5.0)
        try:
            rospy.loginfo(f'************UPDATING apriltag first elem: {self.apriltag_first_elem}')
            self.trans = self.tfBuffer.lookup_transform('pedestal', self.apriltag_first_elem,rospy.Time(0))
            cube_pose = geometry_msgs.msg.Pose()
            cube_pose.position = self.trans.transform.translation
            cube_pose.orientation = self.trans.transform.rotation
            self.calc_grasp_poses(cube_pose)

            self.find_current_apriltag_home_pose()
            self.calc_release_poses(self.apriltagHomePose)
            # self.apriltag_first_elem = self.apriltag_home_list[0]

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rate.sleep()


    def calc_pose(self, cube_pose, pose):
        '''
        This method converts the cube pose found from the tf tree with the offset defined in the config/reachy_grasp.yaml file
        '''
        best_pose = geometry_msgs.msg.Pose()
        best_pose.position.x = cube_pose.position.x + pose[0]
        best_pose.position.y = cube_pose.position.y + pose[1]
        best_pose.position.z = cube_pose.position.z + pose[2]

        quat = quaternion_from_euler (0.0, -1.57, 0.0)

        best_pose.orientation.x = quat[0]       
        best_pose.orientation.y = quat[1]
        best_pose.orientation.z = quat[2]
        best_pose.orientation.w = quat[3]

        return best_pose

    def find_current_apriltag_home_pose(self):
        '''
        This method updates the current apriltag's home position. self.apriltag_data is a list of the apriltags populated in the apriltag_home.json.
        Once reachy releases the cube in the home location, it pops off that element from the list and updates the
        self.apriltag_first_elem. 
        '''
        self.current_apriltagHome = self.apriltag_data[self.apriltag_first_elem]
        self.apriltagHomePose = Pose()
        self.apriltagHomePose.position.x = self.current_apriltagHome['position']['x']
        self.apriltagHomePose.position.y = self.current_apriltagHome['position']['y']
        self.apriltagHomePose.position.z = self.current_apriltagHome['position']['z']
        q = quaternion_from_euler(0.0, -math.pi*0.5, 0.0)
        self.apriltagHomePose.orientation.x = q[0]
        self.apriltagHomePose.orientation.y = q[1]
        self.apriltagHomePose.orientation.z = q[2]
        self.apriltagHomePose.orientation.w = q[3]

    def calc_grasp_poses(self, cube_pose):
        '''
        This method updates the posese for approaching and extending/grasping when in GRASP mode. The arm approaches 
        the object and then goes to extend/grasping pose
        '''
        self.grasp_approach_pose = self.calc_pose(cube_pose,[self._approach_x_offset,
                                                             self._approach_y_offset,
                                                             self._approach_z_offset])
        self.grasp_pose = self.calc_pose(cube_pose,[self._grasp_x_offset,
                                                    self._grasp_y_offset,
                                                    self._grasp_z_offset])

    def calc_release_poses(self, apriltag_home_pose):
        '''
        This method updates the posese for approaching and extending/grasping when in RELEASE mode. The arm approaches 
        the object and then goes to extend/grasping pose
        '''
        self.release_approach_pose = self.calc_pose(apriltag_home_pose,[self._release_approach_x_offset,
                                                                        self._release_approach_y_offset,
                                                                        self._release_approach_z_offset])
        self.release_pose = self.calc_pose(apriltag_home_pose,[self._release_x_offset,
                                                                self._release_y_offset,
                                                                self._release_z_offset])

    def distance(self, pose1, pose2):
        '''
        This method calculates the distance between two poses
        '''
        dx = pose1.position.x - pose2.position.x
        dy = pose1.position.y - pose2.position.y
        dz = pose1.position.z - pose2.position.z
        return math.sqrt(dx*dx + dy*dy + dz*dz)

    def goToPose(self, pose):
        '''
        This method starts a new thread and executes the plan. While the thread is running,
        it checks if there's any error. Depending on the result of the plan execution, it returns
        an int. The reachy_fsm.py will execute different state depending on the returned int.
        '''

        self.move_group.set_pose_target(pose)
        plan = self.move_group.plan()
        plan_status = plan[0]
        result = 0 # no plan

        if plan_status:
            rospy.loginfo("Plan Found")
            execThread = Thread(target=self.trajectoryLoop, args=(self.move_group, plan[1]))
            self.isExecuting = True
            execThread.start()

            while(self.isExecuting):
                self.hertz.sleep()
                if self.errorIds != None:
                    self.move_group.stop()
                    result = 2
                    self.isExecuting = False
                    rospy.loginfo(self.current_pose)

                # UNCOMMENT BELOW TO HAVE REACHY REPLAN IF CUBE HAS MOVED DURING EXECUTING A PLAN. 
                # THIS WAS COMMENTED OUT DURING THE DEMO
                
                # if self.distance(self.grasp_approach_pose, pose) > self.min_approach_error:
                #     rospy.loginfo("target has moved. aborting current trj.")
                #     # abort current trajectory
                #     self.abort_trajectory()
                #     # trigger new trajectory
                #     self.isExecuting = False
                #     result = 1 # target moved

            execThread.join()

            if self.distance(self.current_pose, pose) <= self.min_approach_error:
                rospy.loginfo("Approach success")
                result = 3 # move to extend state
            else:
                rospy.loginfo("trj successful but not within threshold")
                result = 4
        else:
            rospy.loginfo("Plan NOT Found")
            self.hertz.sleep()

        return result

    def trajectoryLoop(self, group, plan):
        group.execute(plan, wait = True)
        self.isExecuting = False

    def setTelemetry(self, telem: Telemetry):
        dxlIds = telem.dxl_ids
        errorBits = telem.error_bits
        errorIds = []
        self.errorIds = None
        thereWereErrors = False
        for i in range(0, len(dxlIds)):
            if errorBits[i] != 0:
                thereWereErrors = True
                errorIds.append(dxlIds[i])
        if thereWereErrors:
            self.errorIds = errorIds


    def abort_trajectory(self):
        '''
        This method aborts the trj
        '''
        self.move_group.stop()
        self.move_group.clear_pose_targets()
        rospy.loginfo("aborted the trj")
        self.trajectory_in_progress = False

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
