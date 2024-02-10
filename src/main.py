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

class Constants:
        WHEEL_DIAMETER_IN = 4
        WHEEL_CIRCUMFERENCE_IN = WHEEL_DIAMETER_IN * math.pi

        DRIVE_BASE_RADIUS_IN = 7.75 # TODO
        DRIVE_BASE_CIRCUMFERENCE_IN = DRIVE_BASE_RADIUS_IN * 2 * math.pi

        MOTOR_MAX_SPEED_RPM = 200

        LOOP_PERIOD_MSECS = 20

        # where phi is the angle between the x/y axis and the wheel vectors, which is always some multiple of pi/4
        SEC_PHI = 2 / math.sqrt(2)

        FUDGE_ODOMETRY_COEFF = 3

class Odometry:
    def __init__(self, x: float, y: float, theta: float, loopPeriodMSecs: float) -> None:
        self.xMeters = x
        self.yMeters = y
        self.rotationRad = theta
        self.loopPeriodSecs = loopPeriodMSecs / 1000
        self.timer = Timer()

    def update(self, xVelocityMetersPerSec: float, yVelocityMetersPerSec: float, headingRad: float):
        self.xMeters += (xVelocityMetersPerSec * self.timer.value())
        self.yMeters += (yVelocityMetersPerSec * self.timer.value())
        self.rotationRad = headingRad
        self.timer.reset()
    
    def getXMeters(self):
        return self.xMeters
    
    def getYMeters(self):
        return self.yMeters
    
    def getRotationRad(self):
        return self.rotationRad

class Drive:
    flDrive = Motor(DevicePorts.FL_DRIVE)
    frDrive = Motor(DevicePorts.FR_DRIVE)
    blDrive = Motor(DevicePorts.BL_DRIVE)
    brDrive = Motor(DevicePorts.BR_DRIVE)

    gyro = Inertial(DevicePorts.GYRO)

    odometry = Odometry(0, 0, 0, Constants.LOOP_PERIOD_MSECS)

    def __init__(self):
        self.gyro.calibrate()
        while self.gyro.is_calibrating():
            pass
        print("GYRO CALIBRATED")

    @staticmethod
    def __metersPerSecToRPM(speedMetersPerSec):
        return (speedMetersPerSec * 39.3701 * 60) / (Constants.WHEEL_CIRCUMFERENCE_IN)
    
    @staticmethod
    def __rpmToMetersPerSecond(speedMetersPerSec):
        return (speedMetersPerSec * Constants.WHEEL_CIRCUMFERENCE_IN) / (39.3701 * 60)
    

    @staticmethod
    def __radPerSecToRPM(speedRadPerSec):
        return (speedRadPerSec * Constants.WHEEL_CIRCUMFERENCE_IN * 60) / (2 * math.pi * Constants.WHEEL_CIRCUMFERENCE_IN)

    def periodic(self):
        directionRad = self.getActualDirectionOfTravelRad()
        speedMetersPerSecond = self.getActualSpeedMetersPerSec()
        
        self.odometry.update(
            speedMetersPerSecond * math.sin(directionRad),
            speedMetersPerSecond * math.cos(directionRad),
            self.gyro.heading(RotationUnits.REV) * 2 * math.pi
        )

    def applyDesaturated(self, flSpeedRPM, frSpeedRPM, blSpeedRPM, brSpeedRPM):
        fastestSpeedRPM = max(abs(flSpeedRPM), abs(frSpeedRPM), abs(blSpeedRPM), abs(brSpeedRPM))
        if(fastestSpeedRPM > Constants.MOTOR_MAX_SPEED_RPM):
            ratio = Constants.MOTOR_MAX_SPEED_RPM / fastestSpeedRPM

            flSpeedRPM *= ratio
            frSpeedRPM *= ratio
            blSpeedRPM *= ratio
            brSpeedRPM *= ratio
        
        self.flDrive.spin(FORWARD, flSpeedRPM)
        self.frDrive.spin(FORWARD, frSpeedRPM)
        self.blDrive.spin(FORWARD, blSpeedRPM)
        self.brDrive.spin(FORWARD, brSpeedRPM)

    def applySpeeds(self, directionRad: float, translationSpeedMetersPerSec: float, rotationSpeedRadPerSec: float, fieldOriented: bool):
        translationRPM = self.__metersPerSecToRPM(translationSpeedMetersPerSec)
        rotationRPM = -self.__radPerSecToRPM(rotationSpeedRadPerSec)

        # offset lateral direction by gyro heading for field-oriented control
        if fieldOriented:
            directionRad += self.gyro.heading(RotationUnits.REV) * 2 * math.pi

        # find the x and y components of the direction vector mapped to a wheel vector
        coeffRPM = Constants.SEC_PHI * translationRPM
        xProjectionRPM = coeffRPM * math.sin(directionRad)
        yProjectionRPM = coeffRPM * math.cos(directionRad)

        self.applyDesaturated(
            rotationRPM - (xProjectionRPM - yProjectionRPM),
            rotationRPM - (xProjectionRPM + yProjectionRPM),
            rotationRPM + (xProjectionRPM + yProjectionRPM),
            rotationRPM + (xProjectionRPM - yProjectionRPM)
        )

    def getActualDirectionOfTravelRad(self, fieldOriented = True):
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

        direction = math.atan2(ySumVectors, xSumVectors)

        if fieldOriented:
            direction += self.gyro.heading(RotationUnits.REV) * 2.0 * math.pi

        return direction
    
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

        magnitude = magnitude = self.__rpmToMetersPerSecond(math.sqrt(xSumVectors**2 + ySumVectors**2) / 4)

        return magnitude



























drive = Drive()
timer = Timer()

# wheelPos = 0

def robotPeriodic():
    timer.event(robotPeriodic, Constants.LOOP_PERIOD_MSECS)

    xIn = -controller.axis4.position() * 0.01
    yIn = controller.axis3.position() * 0.01
    thetaIn = -controller.axis1.position() * 0.01

    direction = math.atan2(xIn, yIn)
    magnitude = math.sqrt(xIn**2 + yIn**2)

    drive.applySpeeds(direction, magnitude * 1.0, thetaIn * 2 * math.pi, True)

    print("X Meters: " + str(drive.odometry.getXMeters()), "Y Meters: " + str(drive.odometry.getYMeters()), "Rotation Radians: " + str(drive.odometry.getRotationRad()))

    drive.periodic()


robotPeriodic()