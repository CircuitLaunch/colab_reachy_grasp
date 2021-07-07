import rospy
import math
from colab_reachy_control.srv import ReadRegisters, ReadRegistersRequest, ReadRegistersResponse, WriteRegisters, WriteRegistersRequest, WriteRegistersResponse

EEPROM_FIRMWARE_VERSION = 2
EEPROM_RETURN_DELAY_TIME = 5
EEPROM_CW_ANGLE_LIMIT = 6
EEPROM_CCW_ANGLE_LIMIT = 8
EEPROM_TEMPERATURE_LIMIT = 11
EEPROM_MIN_VOLTAGE_LIMIT = 12
EEPROM_MAX_VOLTAGE_LIMIT = 13
EEPROM_MAX_TORQUE = 14
EEPROM_STATUS_RETURN_LEVEL = 16
EEPROM_ALARM_LED = 17
EEPROM_SHUTDOWN = 18

MX_EEPROM_MULTI_TURN_OFFSET = 20
MX_EEPROM_RESOLUTION_DIVIDER = 22

RAM_TORQUE_ENABLE = 24
RAM_LED = 25
RAM_GOAL_POSITION = 30
RAM_MOVING_SPEED = 32
RAM_TORQUE_LIMIT = 34
RAM_PRESENT_POSITION = 36
RAM_PRESENT_SPEED = 38
RAM_PRESENT_LOAD = 40
RAM_PRESENT_VOLTAGE = 42
RAM_PRESENT_TEMPERATURE = 43
RAM_REGISTERED = 44
RAM_MOVING = 46
RAM_LOCK = 47
RAM_PUNCH = 48

AX_RAM_CW_COMPLIANCE_MARGIN = 26
AX_RAM_CCW_COMPLIANCE_MARGIN = 27
AX_RAM_CW_COMPLIANCE_SLOPE = 28
AX_RAM_CCW_COMPLIANCE_SLOPE = 29

EX_RAM_SENSED_CURRENT = 56

MX_RAM_D_GAIN = 26
MX_RAM_I_GAIN = 27
MX_RAM_P_GAIN = 28
MX_RAM_REALTIME_TICK = 50
MX_RAM_GOAL_ACCELERATION = 73

MX64_RAM_CURRENT = 68
MX64_RAM_TORQUE_CTRL_MODE_ENABLE = 70
MX64_RAM_GOAL_TORQUE = 71

ERROR_BIT_VOLTAGE = 1
ERROR_BIT_ANGLE_LIMIT = 2
ERROR_BIT_OVERHEATING = 4
ERROR_BIT_RANGE = 8
ERROR_BIT_CHECKSUM = 16
ERROR_BIT_OVERLOAD = 32
ERROR_BIT_INSTRUCTION = 64

class DXLProxy:
    def __init__(self):
        self.readRegSrvProx = rospy.ServiceProxy('dxl_read_registers_service', ReadRegisters)
        self.writeRegSrvProx = rospy.ServiceProxy('dxl_write_registers_service', WriteRegisters)

    def readRegisters(self, ids, registers):
        req = ReadRegistersRequest()
        req.dxl_ids = ids
        req.registers = registers
        resp = self.readRegSrvProx(req)
        convertedValues = [self.convertFromRead(id, register, value) for id, register, value in zip(ids, registers, resp.values)]
        return resp.values, resp.results, resp.error_bits

    def writeRegisters(self, ids, registers, values):
        convertedValues = [self.convertForWrite(id, register, value) for id, register, value in zip(ids, registers, values)]
        req = WriteRegistersRequest()
        req.dxl_ids = ids
        req.registers = registers
        req.values = convertedValues
        resp = self.writeRegSrvProx(req)
        return resp.results, resp.error_bits

    def convertForWrite(self, id, register, value):
        convertedValue = value
        if register in [EEPROM_CW_ANGLE_LIMIT, EEPROM_CCW_ANGLE_LIMIT]:
            convertedValue = self.fromAngle(id, value)
        if register == RAM_GOAL_POSITION:
            convertedValue = self.fromAngle(id, value * self.polarity(id))
        if register == RAM_MOVING_SPEED:
            convertedValue = int(value * 60.0 / self.movingSpeedResolution(id))
        return convertedValue

    def convertFromRead(self, id, register, value):
        convertedValue = value
        if register in [EEPROM_CW_ANGLE_LIMIT, EEPROM_CCW_ANGLE_LIMIT]:
            convertedValue = self.toAngle(id, value) * self.polarity(id)
        if register in [RAM_GOAL_POSITION, RAM_PRESENT_POSITION]:
            convertedValue = self.toAngle(id, value) * self.polarity(id)
        if register == RAM_MOVING_SPEED:
            convertedValue = float(value) * self.movingSpeedResolution(id) / 60.0
        return convertedValue

    def fromAngle(self, id, value):
        return int(self.centerOffset(id) + (value + self.offset(id)) / self.stepResolution(id))

    def toAngle(self, id, value):
        return float((value - self.centerOffset(id)) * self.stepResolution(id) - self.offset(id))

    def stepResolution(self, id):
        if id in [10, 20, 11, 21, 12, 22, 13, 23, 15, 25]: # MX
            return math.radians(0.088)
        #if id in [12, 22, 14, 24, 17, 27]: # AX
        #    pass
        return math.radians(0.29)

    def centerOffset(self, id):
        if id in [10, 20, 11, 21, 12, 22, 13, 23, 15, 25]: # MX
            return 2048
        #if id in [12, 22, 14, 24, 17, 27]: # AX
        #    pass
        return 512

    def movingSpeedResolution(self, id):
        if id in [10, 20, 11, 21, 12, 22, 13, 23, 15, 25]:
            return 0.114
        return 0.111

    def offset(self, id):
        if id in [10, 11]:
            return math.radians(-90.0)
        if id in [20, 21]:
            return math.radians(90.0)
        if id == 24:
            return math.radians(-15.0)
        return 0.0

    def polarity(self, id):
        if id in [16, 20, 27]:
            return 1.0
        return -1.0
