# ---------------------------------------------------------------------------- #
#                                                                              #
# 	Module:       main.py                                                      #
# 	Author:       lgkip                                                        #
# 	Created:      2/6/2024, 5:13:19 PM                                         #
# 	Description:  V5 project                                                   #
#                                                                              #
# ---------------------------------------------------------------------------- #

# Library imports
from vex import *

# Brain should be defined by default
brain = Brain()

controller = Controller()

class DevicePorts:
    FL_DRIVE = Ports.PORT19
    FR_DRIVE = Ports.PORT20
    BL_DRIVE = Ports.PORT10
    BR_DRIVE = Ports.PORT9

    GYRO = Ports.PORT5

class Drive:
    class Constants:
        WHEEL_DIAMETER_IN = 4

        MOTOR_MAX_SPEED_RPM = 200

    flDrive = Motor(DevicePorts.FL_DRIVE)
    frDrive = Motor(DevicePorts.FR_DRIVE)
    blDrive = Motor(DevicePorts.BL_DRIVE)
    brDrive = Motor(DevicePorts.BR_DRIVE)

    gyro = Inertial(DevicePorts.GYRO)

    def __init__(self):
        pass
        
    @staticmethod
    def __metersPerSecToRPM(speedMetersPerSec):
        return (speedMetersPerSec * 39.3701 * 60) / (Drive.Constants.WHEEL_DIAMETER_IN * math.pi)
    
    def applyDesaturated(self, flSpeedRPM, frSpeedRPM, blSpeedRPM, brSpeedRPM):
        fastestSpeedRPM = max(abs(flSpeedRPM), abs(frSpeedRPM), abs(blSpeedRPM), abs(brSpeedRPM))
        if(fastestSpeedRPM > Drive.Constants.MOTOR_MAX_SPEED_RPM):
            flSpeedRPM /= fastestSpeedRPM
            frSpeedRPM /= fastestSpeedRPM
            blSpeedRPM /= fastestSpeedRPM
            brSpeedRPM /= fastestSpeedRPM
        
        self.flDrive.spin(FORWARD, flSpeedRPM)
        self.frDrive.spin(FORWARD, frSpeedRPM)
        self.blDrive.spin(FORWARD, blSpeedRPM)
        self.brDrive.spin(FORWARD, brSpeedRPM)

    SEC_PHI = 2 / math.sqrt(2)
    def translate(self, directionRad: float, speedMetersPerSec: float):
        rpm = self.__metersPerSecToRPM(speedMetersPerSec)

        coeff = self.SEC_PHI * rpm
        xProjection = coeff * math.sin(directionRad)
        yProjection = coeff * math.cos(directionRad)

        # zeroes will be rotation speed
        self.applyDesaturated(
            0 - (xProjection - yProjection),
            0 - (xProjection + yProjection),
            0 + (xProjection + yProjection),
            0 + (xProjection - yProjection)
        )

drive = Drive()

while True:
    drive.translate(math.pi / 8, 0.1)

    # xIn = controller.axis4.position() * 0.01
    # yIn = controller.axis3.position() * 0.01

    # direction = math.atan2(xIn, yIn)
    # # magnitude = math.hypot(xIn, yIn)

    # drive.translate(direction, 100)