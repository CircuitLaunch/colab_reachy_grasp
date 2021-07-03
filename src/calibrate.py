#!/usr/bin/env python3
import math
from os.path import expanduser
import sys
import rospy
import moveit_commander
from geometry_msgs.msg import Pose
from tf.transformations import *
from colab_reachy_control.msg import Telemetry
from colab_reachy_control.srv import Recover, RecoverRequest, Relax, RelaxRequest, Zero, ZeroRequest
from threading import Lock, Thread
import numpy as np
import time

class Calibrator:
    def __init__(self):
        self.tagTopic = rospy.get_param('/calibrate/april_tag_topic')
        self.minx = rospy.get_param('/calibrate/min_x')
        self.maxx = rospy.get_param('/calibrate/max_x')
        self.miny = rospy.get_param('/calibrate/min_y')
        self.maxy = rospy.get_param('/calibrate/max_y')
        self.minz = rospy.get_param('/calibrate/min_z')
        self.maxz = rospy.get_param('/calibrate/max_z')
        self.divx = rospy.get_param('/calibrate/div_x')
        self.divy = rospy.get_param('/calibrate/div_y')
        self.divz = rospy.get_param('/calibrate/div_z')
        self.stepx = (self.maxx - self.minx) / self.divx
        self.stepy = (self.maxy - self.miny) / self.divy
        self.stepz = (self.maxz - self.minz) / self.divz
        self._isExecuting = False
        self._isExecutingLock = Lock()
        self._tagPose = Pose()
        self._tagPoseLock = Lock()
        self._errorIds = None
        self._errorIdsLock = Lock()
        self.hertz = rospy.Rate(30)

    @property
    def isExecuting(self):
        val = False
        with self._isExecutingLock:
            val = self._isExecuting
        return val

    @isExecuting.setter
    def isExecuting(self, val):
        with self._isExecutingLock:
            self._isExecuting = val

    @property
    def errorIds(self):
        val = None
        with self._errorIdsLock:
            val = self._errorIds
        return val

    @errorIds.setter
    def errorIds(self, val):
        with self._errorIdsLock:
            self._errorIds = val

    def getAprilTagPosition(self, side):
        x = 0.0
        y = 0.0
        z = 0.0
        with self._tagPoseLock:
            x = self._tagPose.position.x
            y = self._tagPose.position.y
            z = self._tagPose.position.z
        return x, y, z

    def setAprilTagPose(self, pose):
        with self.tagPoseLock:
            self._tagPose = pose

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

    def createMap(self, side):
        robot = moveit_commander.RobotCommander()
        self.current_group = moveit_commander.MoveGroupCommander(f'{ side }_arm')

        tagSub = rospy.Subscriber(self.tagTopic, Pose, self.setAprilTagPose)
        telemSub = rospy.Subscriber('dxl_telemetry', Telemetry, self.setTelemetry)

        self.zero = rospy.ServiceProxy('zero', Zero)
        self.reachyRelax = rospy.ServiceProxy('relax', Relax)
        self.reachyRecover = rospy.ServiceProxy('recover', Recover)

        restPose = Pose()
        restPose.position.x = 0.0
        restPose.position.y = 0.2 if side == 'left' else -0.2
        restPose.position.z = 0.354
        q = quaternion_from_euler(0.0, 0.0, 0.0)
        restPose.orientation.x = q[0]
        restPose.orientation.y = q[1]
        restPose.orientation.z = q[2]
        restPose.orientation.w = q[3]

        readyPose = Pose()
        readyPose.position.x = 0.1
        readyPose.position.y = 0.2 if side == 'left' else -0.2
        readyPose.position.z = 0.65
        q = quaternion_from_euler(0.0, -math.pi*0.5, 0.0)
        readyPose.orientation.x = q[0]
        readyPose.orientation.y = q[1]
        readyPose.orientation.z = q[2]
        readyPose.orientation.w = q[3]

        self.dxlIds = [ i + (20 if side == 'left' else 10) for i in range(8)]

        while self.goToPose(restPose) == 2:
            self.recover(side)

        time.sleep(1.0)

        while(self.goToPose(readyPose) == 2):
            self.recover(side)

        time.sleep(1.0)

        map1d = np.array([None] * (self.divx+1) * (self.divy+1) * (self.divz+1))
        map = map1d.reshape(self.divx+1, self.divy+1, self.divz+1)
        pose = Pose()
        for k in range(0, self.divz + 1):
            z = self.minz + self.stepz * k
            for j in range(0, self.divy + 1):
                y = self.miny + self.stepy * j
                if math.abs(y) < 0.15:
                    if side == 'left':
                        q = quaternion_from_euler(0.0, -math.pi*0.5, -math.pi*0.25)
                    if side == 'right':
                        q = quaternion_from_euler(0.0, -math.pi*0.5, math.pi*0.25)
                else:
                    if side == 'left':
                        q = quaternion_from_euler(0.0, -math.pi*0.5, -math.pi*0.125)
                    if side == 'right':
                        q = quaternion_from_euler(0.0, -math.pi*0.5, math.pi*0.125)
                for i in range(0, self.divx + 1):
                    x = self.minx + self.stepx * i
                    pose.position.x = x
                    pose.position.y = y
                    pose.position.z = z
                    pose.orientation.x = q[0]
                    pose.orientation.y = q[1]
                    pose.orientation.z = q[2]
                    pose.orientation.w = q[3]
                    rospy.loginfo(f'({i}, {j}, {k}).({x}, {y}, {z}): Attempting plan and trajectory')
                    while(True):
                        result = self.goToPose(pose)
                        if result == 0 or result == 1:
                            break;
                        if result == 2:
                            recovering = True
                            while(recovering):
                                rospy.loginfo('Actuator error, recovering')
                                if self.recover(side) == 'success':
                                    rospy.loginfo('Recovered from actuator error, retrying')
                                    recovering = False
                                else:
                                    if self.errorIds == None or len(self.errorIds) == 0:
                                        recovering = False
                                    else:
                                        rospy.loginfo('Failed to recover from actuator error, waiting 10 seconds, then trying to recover again')
                                        time.sleep(10.0)
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

        time.sleep(1.0)

        while self.goToPose(readyPose) == 2:
            self.recover(side)

        time.sleep(1.0)

        while self.goToPose(restPose) == 2:
            self.recover(side)

        relaxReq = RelaxRequest()
        relaxReq.side = side
        self.reachyRelax(relaxReq)
        return map

    def goToPose(self, pose):
        self.current_group.set_pose_target(pose)
        plan = self.current_group.plan()
        result = 1
        if plan[0]:
            execThread = Thread(target=self.trajectoryLoop, args=(self.current_group, plan[1]))
            self.isExecuting = True
            result = 0
            execThread.start()
            # Loop here testing for overloads
            while(self.isExecuting):
                self.hertz.sleep()
                if self.errorIds != None:
                    self.current_group.stop()
                    result = 2
                    self.isExecuting = False
            execThread.join()
        else:
            self.hertz.sleep()

        return result

    def trajectoryLoop(self, group, plan):
        group.execute(plan, wait = True)
        self.isExecuting = False

    def recover(self, side):
        # Set reachy into compliance mode
        relaxReq = RelaxRequest()
        relaxReq.side = side
        self.reachyRelax(relaxReq)
        # Delay

        time.sleep(10.0)

        # Recover from errors
        recovReq = RecoverRequest()
        if self.errorIds != None:
            recovReq.dxl_ids = self.errorIds
        else:
            recovReq.dxl_ids = self.dxlIds
        result = self.reachyRecover(recovReq).result
        self.errorIds = None
        return result

def main():
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('calibrate');

    side = rospy.get_param('/calibrate/side')
    mapSavePath = rospy.get_param('/calibrate/save_file_path')

    calibrator = Calibrator()

    rospy.loginfo('************************************************************')
    rospy.loginfo(f'Calibrating {side} side')
    map = calibrator.createMap(side)
    rospy.loginfo(f'Calibration complete for {side} side')

    rospy.loginfo('------------------------------------------------------------')
    rospy.loginfo(f'Saving map for {side} side to {mapSavePath}')
    rospy.loginfo('------------------------------------------------------------')

    f = open(expanduser(mapSavePath), 'wt')
    map.tofile(f, sep=',', format='%s')
    f.close()
    rospy.loginfo('************************************************************')

if __name__ == '__main__':
    main()
 
