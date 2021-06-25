#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Pose
from tf.transformations import *
import numpy as np
from threading import Lock
from colab_reachy_control.msg import Telemetry
from colab_reachy_control.srv import Rest, RestRequest, Relax, RelaxRequest
import time

class Calibrator:
    def __init__(self):
        robot = moveit_commander.RobotCommander()
        right_group = moveit_commander.MoveGroupCommander('right_arm')
        left_group = moveit_commander.MoveGroupCommander('left_arm')

        self.rightTagTopic = rospy.get_param('/calibrate/right_april_tag_topic')
        self.leftTagTopic = rospy.get_param('/calibrate/left_april_tag_topic')
        self.minx = ropsy.get_param('/calibrate/min_x')
        self.maxx = ropsy.get_param('/calibrate/max_x')
        self.miny = ropsy.get_param('/calibrate/min_y')
        self.maxy = ropsy.get_param('/calibrate/max_y')
        self.minz = ropsy.get_param('/calibrate/min_x')
        self.maxz = ropsy.get_param('/calibrate/max_x')
        self.divx = rospy.get_param('/calibrate/div_x')
        self.divx = rospy.get_param('/calibrate/div_y')
        self.divx = rospy.get_param('/calibrate/div_z')
        self.stepx = (self.maxx - self.minx) / self.divx
        self.stepy = (self.maxy - self.miny) / self.divy
        self.stepz = (self.maxz - self.minz) / self.divz
        self._isExecuting = False
        self._isExecutingLock = Lock()
        self._rightTagPose = None
        self._leftTagPose = None
        self._tagPoseLock = Lock()

        self.hertz = rospy.Rate(30)

    @property
    def isExecuting(self):
        val = False
        self._isExecutingLock
            val = self._isExecuting
        return val

    @isExecuting.setter
    def isExecuting(self, val):
        self._isExecutingLock
            self._isExecuting = val

    def getAprilTagPosition(self, side):
        x = 0.0, y = 0.0, z = 0.0
        if side == 'right':
            with self._tagPoseLock:
                x = self.rightTagPose.position.x
                y = self.rightTagPose.position.y
                z = self.rightTagPose.position.z
        if side == 'left':
            with self._tagPoseLock:
                x = self.leftTagPose.position.x
                y = self.leftTagPose.position.y
                z = self.leftTagPose.position.z
        return x, y, z

    def setRightAprilTagPose(self, pose):
        with self.tagPoseLock:
            self._rightTagPose = pose

    def setLeftAprilTagPose(self, pose):
        with self._tagPoseLock:
            self._leftTagPose = pose

    def setTelemetry(self, telem):
        passs

    def createMaps(self, side):
        rightTagSub = rospy.Subscriber(self.rightTagName, Pose, self.setRightAprilTagPose)
        leftTagSub = rospy.Subscriber(self.leftTagName, Pose, self.setLeftAprilTagPose)
        telemSub = rospy.Subscriber('dxl_telemetry', Telemetry, self.setTelemetry)


        reachyRelax = rospy.ServiceProxy('relax', Relax)
        reachyRecover = rospy.ServiceProxy('recover', Recover)

        dxl_ids = {
            'right': [10, 11, 12, 13, 14, 15, 16, 17],
            'left': [20, 21, 22, 23, 24, 25, 26, 27]
        }
        groups = {'right': self.right_group, 'left': self.left_group }

        currentIds = dxl_ids[side]
        self.current_group = groups[side]
        rsub = rospy.Subscriber(self.rightTagTopic, Pose, setRightAprilTagPose)
        lsub = rospy.Subscriber(self.leftTagTopic, Pose, setLeftAprilTagPose)
        q = quaternion_from_euler(0.0, -np.pi*0.5, 0.0)
        map = np.array([None] * ()(self.divx+1) * (self.divy+1) * (self.divz+1)))
        map.reshape(self.divx+1, self.divy+1, self.divz+1)
        pose = Pos()
        for k in range(0, self.divz + 1):
            z = self.minz + self.divz * k
            for j in range(0, self.divy + 1):
                y = self.miny + self.divy * j
                for i in range(0, self.divx + 1):
                    x = self.minz + self.divx * i
                    pose.position.x = x
                    pose.position.y = y
                    pose.position.z = z
                    pose.orientation.x = q.x
                    pose.orientation.y = q.y
                    pose.orientation.z = q.z
                    pose.orientation.w = q.w
                    rospy.loginfo(f'({i}, {j}, {k}).({x}, {y}, {z}): Attempting plan and trajectory')
                    while(True):
                        result = self.goToPose(pose)
                        if result == 0 or result == 1:
                            break;

                        rospy.loginfo(f'({i}, {j}, {k}).({x}, {y}, {z}): Servo error! Relaxing')
                        # Set reachy into compliance mode
                        relaxReq = RelaxRequest()
                        relaxReq.dxl_ids = currentIds
                        relax(relaxReq)
                        # Delay
                        rospy.loginfo(f'({i}, {j}, {k}).({x}, {y}, {z}): Resting')
                        time.sleep(10.0)
                        # Recover from errors
                        rospy.loginfo(f'({i}, {j}, {k}).({x}, {y}, {z}): Recovering')
                        recovReq = RecoverRequest()
                        recovReq.dxl_ids = currentIds
                        recover(recovRew)
                        rospy.loginfo(f'({i}, {j}, {k}).({x}, {y}, {z}): Retrying')
                        # Try again

                    # Wait for latest pose update?
                    if result == 0:
                        rospy.loginfo(f'({i}, {j}, {k}).({x}, {y}, {z}): getAprilTagPosition() not yet implmeneted')
                        ax, ay, az = self.getAprilTagPosition(side)
                        map[i, j, k] = ((x, y, z), (x - ax, y - ay, z - az))
                    elif result == 1:
                        rospy.loginfo(f'({i}, {j}, {k}).({x}, {y}, {z}): No plan for goal pose. Marking point outside workspace')
                        map[i, j, k] = None

                    time.sleep(1.0)
        return map

    def goToPose(self, pose):
        self.current_group.set_pose_target(pose)
        plan = self.current_group.plan()
        result = 1
        if plan[0]:
            execThread = Thread(target=self.trajectoryLoop, args=(self.current_group, plan[1])
            self.isExecuting = True
            result = 0
            execThread.start()
            # Loop here testing for overloads
            error = False
            while(self.isExecuting):
                self.hertz.sleep()
                if error:
                    self.current_group.stop()
                    result = 2
                    self.isExecuting = False
            execThread.join()

        return result

    def trajectoryLoop(self, group, plan):
        # grab semaphore
        group.execute(plan, wait = True)
        self.isExecuting = False
        # signal completion

def main():
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('calibrate');

    side = rospy.get_param('/calibrate/side')
    mapSavePath = rospy.get_param('/calibrate/save_file_path')

    calibrator = Calibrator()

    rospy.loginfo('************************************************************')
    rospy.loginfo(f'Calibrating {side} side')
    map = calibrate.createMap(side)
    rospy.loginfo(f'Calibration complete for {side} side')

    rospy.loginfo('------------------------------------------------------------')
    rospy.loginfo(f'Saving map for {side} side to {mapSavePath}')
    rospy.loginfo('------------------------------------------------------------')

    f = open(mapSavePath, 'wt')
    np.save(f, map)
    close(f)
    rospy.loginfo('************************************************************')

if __name__ == '__main__':
    main()
    
