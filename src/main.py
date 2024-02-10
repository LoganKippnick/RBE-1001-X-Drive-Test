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

    def applyDesaturatedDistance(self, flSpeedRPM, frSpeedRPM, blSpeedRPM, brSpeedRPM, distanceDelta, rotationDelta):
        fastestSpeedRPM = max(abs(flSpeedRPM), abs(frSpeedRPM), abs(blSpeedRPM), abs(brSpeedRPM))
        if(fastestSpeedRPM > Drive.Constants.MOTOR_MAX_SPEED_RPM):
            ratio = Drive.Constants.MOTOR_MAX_SPEED_RPM / fastestSpeedRPM

            flSpeedRPM *= ratio
            frSpeedRPM *= ratio
            blSpeedRPM *= ratio
            brSpeedRPM *= ratio
        
        # TODO: CHANGE THIS TO SPIN FOR AND DO CALCULATIONS FOR DISTANCE FROM WHEELS
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
        coeff = self.SEC_PHI * translationRPM
        xProjection = coeff * math.sin(directionRad)
        yProjection = coeff * math.cos(directionRad)

        self.applyDesaturated(
            rotationRPM - (xProjection - yProjection),
            rotationRPM - (xProjection + yProjection),
            rotationRPM + (xProjection + yProjection),
            rotationRPM + (xProjection - yProjection)
        )

    def applyDistance(self, directionRad: float, translationSpeedMetersPerSec: float, rotationSpeedRadPerSec: float, fieldOriented: bool, distanceDelta: float, rotationDelta: float):
        translationRPM = self.__metersPerSecToRPM(translationSpeedMetersPerSec)
        rotationRPM = -self.__radPerSecToRPM(rotationSpeedRadPerSec)

        # offset lateral direction by gyro heading for field-oriented control
        if fieldOriented:
            directionRad += self.gyro.heading(RotationUnits.REV) * 2 * math.pi

        # find the x and y components of the direction vector mapped to a wheel vector
        coeff = self.SEC_PHI * translationRPM
        xProjection = coeff * math.sin(directionRad)
        yProjection = coeff * math.cos(directionRad)

        self.applyDesaturatedDistance(
            rotationRPM - (xProjection - yProjection),
            rotationRPM - (xProjection + yProjection),
            rotationRPM + (xProjection + yProjection),
            rotationRPM + (xProjection - yProjection),
            distanceDelta,
            rotationDelta
        )

    def getActualDirectionOfTravelRad(self):
        xLeftVectors = self.flDrive.velocity() * math.cos(7 * math.pi / 4) + self.blDrive.velocity() * math.cos(math.pi / 4)
        yLeftVectors = self.flDrive.velocity() * math.sin(7 * math.pi / 4) + self.blDrive.velocity() * math.sin(math.pi / 4)

        directionLeftVectors = math.atan2(xLeftVectors, yLeftVectors)
        magnitudeLeftVectors = math.sqrt(xLeftVectors**2 + yLeftVectors**2)

        xRightVectors = self.frDrive.velocity() * math.cos(5 * math.pi / 4) + self.brDrive.velocity() * math.cos(3 * math.pi / 4)
        yRightVectors = self.frDrive.velocity() * math.sin(5 * math.pi / 4) + self.brDrive.velocity() * math.sin(3 * math.pi / 4)

        directionRightVectors = math.atan2(xRightVectors, yRightVectors)
        magnitudeRightVectors = math.sqrt(xRightVectors**2 + yRightVectors**2)

        xSumVectors = magnitudeLeftVectors * math.cos(directionLeftVectors) + magnitudeRightVectors * math.cos(directionRightVectors)
        ySumVectors = magnitudeLeftVectors * math.sin(directionLeftVectors) + magnitudeRightVectors * math.sin(directionRightVectors)

        directionSumVectors = math.atan2(xSumVectors, ySumVectors)

        return directionSumVectors
    
    def getActualSpeedMetersPerSec(self):
        # TODO same as above but return magnitude divided by 4, but make sure above works first
        pass

drive = Drive()

while True:
    xIn = -controller.axis4.position() * 0.01
    yIn = controller.axis3.position() * 0.01
    thetaIn = -controller.axis1.position() * 0.01

    direction = math.atan2(xIn, yIn)
    magnitude = math.sqrt(xIn**2 + yIn**2)

    drive.applySpeeds(direction, magnitude * 1.0, thetaIn * 2 * math.pi, True)