#!/usr/bin/env python3
import math
import time
import tf2_ros
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from apriltag_ros.msg import AprilTagDetection, AprilTagDetectionArray
from math import pi, tau, dist, fabs, cos
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
from threading import Lock
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
        rospy.init_node('move_group_python_interface_tutorial', anonymous=True)

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

    # def execute_to_goal(self, msg):
    #   appraoch_to_goal(self, msg)
    #   go_to_pose_goal(self, msg)

    def go_to_pose_goal(self, msg):

        #initialize cube position and looping rate
        rate = rospy.Rate(5.0)
        # rate2 = rospy.Rate(1)
        cube_x = 0
        cube_y = 0
        cube_z = 0


        # rate2.sleep()
        #attempts to get the latest transform of the cube from world
        try:
            trans = self.tfBuffer.lookup_transform('pedestal', 'cube2',rospy.Time(0))
            # cube_x = trans.transform.translation.x
            # cube_y = trans.transform.translation.y
            # cube_z = trans.transform.translation.z
            # print(cube_x,cube_y,cube_z)

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rate.sleep()


        for a in msg.detections:

            if a.id[0] == 5:
                cube_pose = geometry_msgs.msg.Pose()
                cube_pose.position = trans.transform.translation
                cube_pose.orientation = trans.transform.rotation
                self.on_cube_detected(cube_pose)

                '''
                pose_goal = geometry_msgs.msg.Pose()
                pose_goal.position.x = cube_x  - 0.07
                pose_goal.position.y = cube_y
                pose_goal.position.z = cube_z   + 0.145

                quat = quaternion_from_euler (0.0, -1.57, 0.0)


                pose_goal.orientation.x = quat[0]
                pose_goal.orientation.y = quat[1]
                pose_goal.orientation.z = quat[2]
                pose_goal.orientation.w = quat[3]

                self.move_group.set_pose_target(pose_goal)
                rospy.loginfo("approach")
                rospy.loginfo(pose_goal)
                ## Now, we call the planner to compute the plan and execute it.
                print("Executing motion")
                # print(pose_goal)
                plan = self.move_group.go(wait=True)

                # Calling `stop()` ensures that there is no residual movement
                self.move_group.stop()
                # It is always good to clear your targets after planning with poses.
                # Note: there is no equivalent function for clear_joint_value_targets()
                self.move_group.clear_pose_targets()


                rate.sleep()
                rospy.loginfo("GOING TO THE CUBE NOW")


                pose_goal = geometry_msgs.msg.Pose()
                pose_goal.position.x = cube_x + 0.06
                pose_goal.position.y = cube_y
                pose_goal.position.z = cube_z   + 0.145


                quat = quaternion_from_euler (0.0, -1.57, 0.0)


                pose_goal.orientation.x = quat[0]
                pose_goal.orientation.y = quat[1]
                pose_goal.orientation.z = quat[2]
                pose_goal.orientation.w = quat[3]

                rospy.loginfo("graasp")
                rospy.loginfo(pose_goal)
                self.move_group.set_pose_target(pose_goal)

                plan = self.move_group.go(wait=True)
                self.move_group.stop()
                self.move_group.clear_pose_targets()
                '''





    def grasping_state_init(self):
        self._min_approach_error = 0.01     # TODO: determine a suitable minimum approach pose error
        self._min_grip_error = 0.005        # TODO: determine a suitable minimum grip pose error
        self._max_grip_load = 0.2
        
        # ADDED BY LIAM
        self._approach_x_offset = 0.07
        self._approach_y_offset = 0.0
        self._approach_z_offset = 0.145

        self._grasp_x_offset = 0.06
        self._grasp_y_offset = 0.0
        self._grasp_z_offset = 0.145

        # ADDED BY LIAM

        self._settings_mutex = Lock()

        self._grip_load = 0.0
        self._grip_load_mutex = Lock()
        self._abort = False
        self._abort_mutex = Lock()

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

    @min_approach_error
    def min_approach_error(self, val):
        with self._settings_mutex:
            self._min_approach_error = val

    @property
    def min_grip_error(self):
        local_val = 0.0
        with self._settings_mutex:
            local_val = self._min_grip_error
        return local_val

    @min_grip_error
    def min_grip_error(self, val):
        with self._settings_mutex:
            self._min_grip_error = val


    #ADDED BY LIAM
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
    #ADDED BY LIAM

    @property
    def get_cube_pose(self):
        


    @property
    def max_grip_load(self):
        local_val = 1.0
        with self._settings_mutex:
            local_val = self._max_grip_load
        return local_val

    @max_grip_load
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
    def current_pose(self):
        return self.move_group.get_current_pose()
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


# trans = self.tfBuffer.lookup_transform('pedestal', 'cube2',rospy.Time(0))
#             cube_x = trans.transform.translation.x
#             cube_y = trans.transform.translation.y
#             cube_z = trans.transform.translation.z


#                 pose_goal = geometry_msgs.msg.Pose()
#                 pose_goal.position.x = cube_x + 0.06
#                 pose_goal.position.y = cube_y
#                 pose_goal.position.z = cube_z   + 0.145


    def calc_approach_pose(self, cube_pose):
        best_pose = geometry_msgs.msg.Pose()

        # TODO: Calculate the approach pose given the cube pose pose

        # ADDED BY LIAM
        best_pose.position.x = cube_pose.position.x + self.approach_x_offset
        best_pose.position.y = cube_pose.position.y + self.approach_y_offset
        best_pose.position.z = cube_pose.position.z + self.approach_z_offset
        # ADDED BY LIAM

        return best_pose

    def calc_grasp_pose(self, cube_pose):
        best_pose = geometry_msgs.msg.Pose()

        # TODO: Calculate the grasp pose given the cube pose
        
        # ADDED BY LIAM
        best_pose.position.x = cube_pose.position.x + self.grasp_x_offset
        best_pose.position.y = cube_pose.position.y + self.grasp_y_offset
        best_pose.position.z = cube_pose.position.z + self.grasp_z_offset
        # ADDED BY LIAM

        return best_pose

    def on_cube_detected(self, cube_pose):
        self.approach_pose = self.calc_approach_pose(cube_pose)
        self.grasp_pose = self.calc_grasp_pose(cube_pose)

    def distance(self, pose1, pose2):
        dx = pose1.position.x - pose2.position.x
        dy = pose1.position.y - pose2.position.y
        dz = pose1.position.z - pose2.position.z
        return math.sqrt(dx*dx + dy*dy + dz*dz)

    def go_to_pose(self, pose):
        self.trajectory_in_progress = True
        self.move_group.set_pose_target(pose)
        self.move_group.go(wait = False)

    def abort_trajectory(self):
        self.move_group.stop()
        self.move_group.clear_pose_target()
        self.trajectory_in_progress = False

    def tighten_grip(self):
        pass

    def min_grip_load(self):
        return 0.2

    def grasping_state_loop(self):
        self.abort = False
        # Do move to approach pose, move to grasp pose and grasp
        while(True):
            self.spin_rate.sleep()
            if self.abort or rospy.is_shutdown():
                return False
            # Do move to approach pose and move to grasp pose
            while(True):
                self.spin_rate.sleep()
                if self.abort or rospy.is_shutdown():
                    return False
                approach_pose_attained = False
                grasp_pose_attained = False
                # Do move to approach pose
                local_approach_pose = self.approach_pose
                while(True):
                    self.spin_rate.sleep()
                    if self.abort or rospy.is_shutdown():
                        return False
                    ''' # THESE ARE DETERMINED ASYNCHRONOUSLY EVERYTIME THE TAG IS DETECTED
                        # Determine grasp pose
                        # Determine current pose
                    '''
                    # If no trajectory being executed
                    if not self.trajectory_in_progress():
                        # trigger trajectory
                        self.go_to_pose(local_approach_pose)
                    # Else
                    else:
                        # If approach pose has changed
                        if self.distance(self.approach_pose, local_approach_pose) > self.min_approach_error:
                            # abort current trajectory
                            self.abort_trajectory()
                            # trigger new trajectory
                            local_approach_pose = self.approach_pose
                            self.go_to_pose(local_approach_pose)
                    # Repeat move to approach pose while distance between current pose and approach pose > threshold and not abort
                    if self.distance(self.current_pose, local_approach_pose) <= self.min_approach_error:
                        approach_pose_attained = True
                        break
                # Do move to grasp pose
                local_grasp_pose = self.grasp_pose
                while(True):
                    self.spin_rate.sleep()
                    if self.abort or rospy.is_shutdown():
                        return False
                    ''' # THESE ARE DETERMINED ASYNCHRONOUSLY EVERYTIME THE TAG IS DETECTED
                        # Determine grasp pose
                        # Determine current pose
                    '''
                    # If no trajectory being executed
                    if not self.trajectory_in_progress():
                        # trigger trajectory
                        self.go_to_pose(local_grasp_pose)
                    # Else
                    else:
                        # If grasp pose has changed
                        if self.distance(self.grasp_pose, local_grasp_pose) > self.min_grasp_error:
                            # abort current trajectory
                            self.abort_trajectory()
                            break
                    # Repeat move to grasp pose while distance between current pose and grasp pose > threshold and not abort
                    if self.distance(self.current_pose, local_grasp_pose) <= self.min_grasp_error:
                        grasp_pose_attained = True
                        break
                # Repeat move to approach pose and move to grasp pose while distance between current pose and grasp pose > threshold and not abort
                if grasp_pose_attained:
                    break
            local_grasp_pose = self.grasp_pose
            # Do grasp
            local_grasp_flag = False
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
                if self.distance(self.current_pose, local_grasp_pose) > self.min_grasp_error:
                    break
                # set grasped true
                local_grasp_flag = True
                # Repeat while grip load < threshold
                if self.grip_load > self.min_grip_load:
                    break
            # Repeat grasp while not (grasped or abort)
            if local_grasp_flag:
                break
        # Return result
        return True

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
