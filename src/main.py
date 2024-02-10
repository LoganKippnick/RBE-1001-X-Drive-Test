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
        WHEEL_CIRCUMFERENCE_IN = WHEEL_DIAMETER_IN * math.pi

        DRIVE_BASE_RADIUS_IN = 7.75 # TODO
        DRIVE_BASE_CIRCUMFERENCE_IN = DRIVE_BASE_RADIUS_IN * 2 * math.pi

        MOTOR_MAX_SPEED_RPM = 200

    flDrive = Motor(DevicePorts.FL_DRIVE)
    frDrive = Motor(DevicePorts.FR_DRIVE)
    blDrive = Motor(DevicePorts.BL_DRIVE)
    brDrive = Motor(DevicePorts.BR_DRIVE)

    gyro = Inertial(DevicePorts.GYRO)

    def __init__(self):
        self.gyro.calibrate()
        while self.gyro.is_calibrating():
            pass
        print("GYRO CALIBRATED")

    @staticmethod
    def __metersPerSecToRPM(speedMetersPerSec):
        return (speedMetersPerSec * 39.3701 * 60) / (Drive.Constants.WHEEL_CIRCUMFERENCE_IN)
    
    @staticmethod
    def __radPerSecToRPM(speedRadPerSec):
        return (speedRadPerSec * Drive.Constants.DRIVE_BASE_CIRCUMFERENCE_IN * 60) / (2 * math.pi * Drive.Constants.WHEEL_CIRCUMFERENCE_IN)
    
    def applyDesaturated(self, flSpeedRPM, frSpeedRPM, blSpeedRPM, brSpeedRPM):
        fastestSpeedRPM = max(abs(flSpeedRPM), abs(frSpeedRPM), abs(blSpeedRPM), abs(brSpeedRPM))
        if(fastestSpeedRPM > Drive.Constants.MOTOR_MAX_SPEED_RPM):
            ratio = Drive.Constants.MOTOR_MAX_SPEED_RPM / fastestSpeedRPM

            flSpeedRPM *= ratio
            frSpeedRPM *= ratio
            blSpeedRPM *= ratio
            brSpeedRPM *= ratio
        
        self.flDrive.spin(FORWARD, flSpeedRPM)
        self.frDrive.spin(FORWARD, frSpeedRPM)
        self.blDrive.spin(FORWARD, blSpeedRPM)
        self.brDrive.spin(FORWARD, brSpeedRPM)

    # where phi is the angle between the x/y axis and the wheel vectors, which is always some multiple of pi/4
    SEC_PHI = 2 / math.sqrt(2)
    def applySpeeds(self, directionRad: float, translationSpeedMetersPerSec: float, rotationSpeedRadPerSec: float, fieldOriented: bool):
        translationRPM = self.__metersPerSecToRPM(translationSpeedMetersPerSec)
        rotationRPM = -self.__radPerSecToRPM(rotationSpeedRadPerSec)

        # offset lateral direction by gyro heading for field-oriented control
        if fieldOriented:
            directionRad += self.gyro.heading(RotationUnits.REV) * 2 * math.pi

        # find the x and y components of the direction vector mapped to a wheel vector
        coeffRPM = self.SEC_PHI * translationRPM
        xProjectionRPM = coeffRPM * math.sin(directionRad)
        yProjectionRPM = coeffRPM * math.cos(directionRad)

        self.applyDesaturated(
            rotationRPM - (xProjectionRPM - yProjectionRPM),
            rotationRPM - (xProjectionRPM + yProjectionRPM),
            rotationRPM + (xProjectionRPM + yProjectionRPM),
            rotationRPM + (xProjectionRPM - yProjectionRPM)
        )

    def getActualDirectionOfTravelRad(self):
        xFL = self.flDrive.velocity() * math.cos(7 * math.pi / 4)
        yFL = self.flDrive.velocity() * math.sin(7 * math.pi / 4)

        xFR = self.blDrive.velocity() * math.cos(math.pi / 4)
        yFR = self.blDrive.velocity() * math.sin(math.pi / 4)
        
        xBL = self.frDrive.velocity() * math.cos(5 * math.pi / 4)
        yBL = self.frDrive.velocity() * math.sin(5 * math.pi / 4)


        xBR = self.brDrive.velocity() * math.cos(3 * math.pi / 4)
        yBR = self.brDrive.velocity() * math.sin(3 * math.pi / 4)

        xSumVectors = xFL + xFR + xBL + xBR
        ySumVectors = yFL + yFR + yBL + yBR

        magnitudeSumVectors = math.sqrt(xSumVectors * xSumVectors + ySumVectors * ySumVectors) / 4,

        return magnitudeSumVectors
    
    def getActualSpeedMetersPerSec(self):
        xFL = self.flDrive.velocity() * math.cos(7 * math.pi / 4)
        yFL = self.flDrive.velocity() * math.sin(7 * math.pi / 4)

        xFR = self.blDrive.velocity() * math.cos(math.pi / 4)
        yFR = self.blDrive.velocity() * math.sin(math.pi / 4)
        
        xBL = self.frDrive.velocity() * math.cos(5 * math.pi / 4)
        yBL = self.frDrive.velocity() * math.sin(5 * math.pi / 4)

        xBR = self.brDrive.velocity() * math.cos(3 * math.pi / 4)
        yBR = self.brDrive.velocity() * math.sin(3 * math.pi / 4)

        xSumVectors = xFL + xFR + xBL + xBR
        ySumVectors = yFL + yFR + yBL + yBR

        directionSumVectors = math.atan2(ySumVectors, xSumVectors)

        return directionSumVectors

drive = Drive()

while True:
    xIn = -controller.axis4.position() * 0.01
    yIn = controller.axis3.position() * 0.01
    thetaIn = -controller.axis1.position() * 0.01

    direction = math.atan2(xIn, yIn)
    magnitude = math.sqrt(xIn**2 + yIn**2)

    drive.applySpeeds(direction, magnitude * 1.0, thetaIn * 2 * math.pi, True)