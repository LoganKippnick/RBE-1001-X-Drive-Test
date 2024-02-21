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
    
    FL_DRIVE = Ports.PORT20
    FR_DRIVE = Ports.PORT9
    BL_DRIVE = Ports.PORT17
    BR_DRIVE = Ports.PORT10

    GYRO = Ports.PORT5

    FL_LINE = brain.three_wire_port.g
    FR_LINE = brain.three_wire_port.b

class Constants:
        
        LOOP_PERIOD_MS = 20
        
        WHEEL_DIAMETER_IN = 4
        WHEEL_CIRCUMFERENCE_IN = WHEEL_DIAMETER_IN * math.pi

        DRIVE_BASE_RADIUS_IN = 7.75
        DRIVE_BASE_CIRCUMFERENCE_IN = DRIVE_BASE_RADIUS_IN * 2 * math.pi

        MOTOR_MAX_SPEED_RPM = 200

        # where phi is the angle between the x/y axis and the wheel vectors, which is always some multiple of pi/4
        SEC_PHI = 2 / math.sqrt(2)

        DRIVE_TRAIN_IDLE_MODE = BRAKE

        DRIVE_TRANSLATION_KP = 1.5

        DRIVE_ROTATION_KP = 1.5

        DRIVE_MAX_SPEED_METERS_PER_SEC = 0.2

        DRIVE_FIND_LINE_SPEED_METERS_PER_SEC = 0.1

        DRIVE_FOLLOW_LINE_SPEED_METERS_PER_SEC = 0.2

        ODOM_X_POSITIVE_DRIFT = 194 / 200
        ODOM_X_NEGATIVE_DRIFT = 207 / 200
        ODOM_Y_POSITIVE_DRIFT = 180 / 200
        ODOM_Y_NEGATIVE_DRIFT = 211 / 200

        ODOM_Y_DRIFT_PER_POSITIVE_X_TRANSLATION = 5 / 200
        ODOM_Y_DRIFT_PER_NEGATIVE_X_TRANSLATION = -17 / 200

        LINE_REFLECTIVITY_THRESHOLD = 25

        FOLLOW_LINE_KP = 0.001
        FOLLOW_LINE_KD = 0.00003


        LEFT_LINE_Y_METERS = 0 # FIXME
        RIGTH_LINE_Y_METERS = 0 # FIXME



class Odometry:

    def __init__(self, x = 0.0, y = 0.0, theta = 0.0):
        self.xMeters = x
        self.yMeters = y
        self.thetaRad = theta
        
    prevTranslationMeters = 0.0
    def update(self, fieldOrientedTranslationRad: float, translationMeters: float, headingRad: float):
        translationDeltaMeters = abs(translationMeters - self.prevTranslationMeters)
        self.prevTranslationMeters = translationMeters

        # As the wheels approach parallel/orthoginal to the drive direction, we don't need to compensate for carpet drift
        robotRelativeTranslationRad = fieldOrientedTranslationRad + headingRad
        driftCompPercentage = abs(math.cos((2 * robotRelativeTranslationRad) % math.pi))

        xDelta = translationDeltaMeters * math.cos(fieldOrientedTranslationRad)
        yDelta = translationDeltaMeters * math.sin(fieldOrientedTranslationRad)

        if(yDelta > 0.0):
            yDelta *= (Constants.ODOM_Y_POSITIVE_DRIFT ** driftCompPercentage)
        else:
            yDelta *= (Constants.ODOM_Y_NEGATIVE_DRIFT ** driftCompPercentage)

        if(xDelta > 0.0):
            xDelta *= (Constants.ODOM_X_POSITIVE_DRIFT ** driftCompPercentage)
            yDelta += ((xDelta * Constants.ODOM_Y_DRIFT_PER_POSITIVE_X_TRANSLATION) * driftCompPercentage)
        else:
            xDelta *= (Constants.ODOM_X_NEGATIVE_DRIFT ** driftCompPercentage)
            yDelta += ((xDelta * Constants.ODOM_Y_DRIFT_PER_NEGATIVE_X_TRANSLATION) * driftCompPercentage)

        self.xMeters += xDelta
        self.yMeters += yDelta
        self.thetaRad = headingRad
        
class LineSensorArray:
    __prevError = 0.0
    __timer = Timer()

    def __init__(self, leftSensorPort: Triport.TriportPort, rightSensorPort: Triport.TriportPort):
        self.leftSensor = Line(leftSensorPort)
        self.rightSensor = Line(rightSensorPort)
        
        self.periodic()

    def periodic(self):
        self.__timer.event(self.periodic, Constants.LOOP_PERIOD_MS)

        if self.onLine():
            self.__lastSensorOnLine = None
        elif self.rightSensor.reflectivity() > Constants.LINE_REFLECTIVITY_THRESHOLD:
            self.__lastSensorOnLine = RIGHT
        elif self.leftSensor.reflectivity() > Constants.LINE_REFLECTIVITY_THRESHOLD:
            self.__lastSensorOnLine = LEFT

        self.__rate = (self.getError() - self.__prevError) / (Constants.LOOP_PERIOD_MS / 1000)
        self.__prevError = self.getError()

    def hasLine(self):
        '''Line is detected by at least one sensor'''
        return self.rightSensor.reflectivity() > Constants.LINE_REFLECTIVITY_THRESHOLD \
            or self.leftSensor.reflectivity() > Constants.LINE_REFLECTIVITY_THRESHOLD
    
    def onLine(self):
        '''Line is detected by both sensors'''
        return self.rightSensor.reflectivity() > Constants.LINE_REFLECTIVITY_THRESHOLD \
            and self.leftSensor.reflectivity() > Constants.LINE_REFLECTIVITY_THRESHOLD

    def getError(self) -> float:
        return self.rightSensor.reflectivity() - self.leftSensor.reflectivity()
    
    def getRate(self) -> float:
        return self.__rate
    
    def getLastSensorOnLine(self):
        '''Returns RIGHT for right sensor, LEFT for leftsensor, or None if they are both on the line'''
        return self.__lastSensorOnLine


class Drive:

    flDrive = Motor(DevicePorts.FL_DRIVE)
    frDrive = Motor(DevicePorts.FR_DRIVE)
    blDrive = Motor(DevicePorts.BL_DRIVE)
    brDrive = Motor(DevicePorts.BR_DRIVE)

    flDrive.set_stopping(Constants.DRIVE_TRAIN_IDLE_MODE)
    frDrive.set_stopping(Constants.DRIVE_TRAIN_IDLE_MODE)
    blDrive.set_stopping(Constants.DRIVE_TRAIN_IDLE_MODE)
    brDrive.set_stopping(Constants.DRIVE_TRAIN_IDLE_MODE)

    gyro = Inertial(DevicePorts.GYRO)

    odometry = Odometry(0.0, 0.0, 0.0)

    __timer = Timer()

    def __init__(self, heading = 0.0, calibrateGyro = False):
        if calibrateGyro:
            self.gyro.calibrate()
            while self.gyro.is_calibrating():
                pass
            print("GYRO CALIBRATED")
            self.gyro.set_heading(heading)
        
        self.periodic()

    @staticmethod
    def __revolutionsToMeters(revolutions) -> float:
        return (revolutions * Constants.WHEEL_CIRCUMFERENCE_IN) / (39.3701)

    @staticmethod
    def __metersPerSecToRPM(speedMetersPerSec) -> float:
        return (speedMetersPerSec * 39.3701 * 60) / (Constants.WHEEL_CIRCUMFERENCE_IN)
    
    @staticmethod
    def __rpmToMetersPerSecond(speedMetersPerSec) -> float:
        return (speedMetersPerSec * Constants.WHEEL_CIRCUMFERENCE_IN) / (39.3701 * 60)
    
    @staticmethod
    def __radPerSecToRPM(speedRadPerSec) -> float:
        return (speedRadPerSec * Constants.WHEEL_CIRCUMFERENCE_IN * 60) / (2 * math.pi * Constants.WHEEL_CIRCUMFERENCE_IN)
    
    def calcThetaControlRadPerSec(self, targetHeadingRad: float) -> float:
        thetaError = targetHeadingRad - self.odometry.thetaRad

        if thetaError > math.pi:
            thetaError -= 2 * math.pi
        elif thetaError < -math.pi:
            thetaError += 2 * math.pi

        return thetaError * Constants.DRIVE_ROTATION_KP

    def periodic(self):
        self.__timer.event(self.periodic, Constants.LOOP_PERIOD_MS)

        self.odometry.update(
            self.getActualDirectionOfTravelRad(),
            self.getDistanceTraveledMeters(),
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
        
        self.flDrive.spin(FORWARD, round(flSpeedRPM, 4))
        self.frDrive.spin(FORWARD, round(frSpeedRPM, 4))
        self.blDrive.spin(FORWARD, round(blSpeedRPM, 4))
        self.brDrive.spin(FORWARD, round(brSpeedRPM, 4))

    def stop(self):
        self.flDrive.stop()
        self.frDrive.stop()
        self.blDrive.stop()
        self.brDrive.stop()

    def applySpeeds(self, directionRad: float, translationSpeedMetersPerSec: float, rotationSpeedRadPerSec: float, fieldOriented = True):
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

    def applySpeedsCartesian(self, xSpeedMetersPerSec: float, ySpeedMetersPerSec: float, rotationSpeedRadPerSec: float, fieldOriented = True):
        direction = math.atan2(ySpeedMetersPerSec, xSpeedMetersPerSec)
        magnitude = math.sqrt(xSpeedMetersPerSec ** 2 + ySpeedMetersPerSec ** 2)
        self.applySpeeds(direction, magnitude, rotationSpeedRadPerSec, fieldOriented)

    def getActualDirectionOfTravelRad(self, fieldOriented = True) -> float:
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
            direction -= self.gyro.heading(RotationUnits.REV) * 2 * math.pi

        return direction
    
    def getDistanceTraveledMeters(self):
        xFL = self.__revolutionsToMeters(self.flDrive.position()) * math.cos(7 * math.pi / 4)
        yFL = self.__revolutionsToMeters(self.flDrive.position()) * math.sin(7 * math.pi / 4)

        xFR = self.__revolutionsToMeters(self.blDrive.position()) * math.cos(math.pi / 4)
        yFR = self.__revolutionsToMeters(self.blDrive.position()) * math.sin(math.pi / 4)
        
        xBL = self.__revolutionsToMeters(self.frDrive.position()) * math.cos(5 * math.pi / 4)
        yBL = self.__revolutionsToMeters(self.frDrive.position()) * math.sin(5 * math.pi / 4)

        xBR = self.__revolutionsToMeters(self.brDrive.position()) * math.cos(3 * math.pi / 4)
        yBR = self.__revolutionsToMeters(self.brDrive.position()) * math.sin(3 * math.pi / 4)

        xSumVectors = xFL + xFR + xBL + xBR
        ySumVectors = yFL + yFR + yBL + yBR

        magnitude = magnitude = self.__rpmToMetersPerSecond(math.sqrt(xSumVectors**2 + ySumVectors**2) / 4)

        return magnitude
    
    def getActualSpeedMetersPerSec(self) -> float:
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

        magnitude = self.__rpmToMetersPerSecond(math.sqrt(xSumVectors**2 + ySumVectors**2) / 4)

        return magnitude
    
    def driveToPosition(self, xMeters: float, yMeters: float, headingRad: float):
        xError = xMeters - self.odometry.xMeters
        yError = yMeters - self.odometry.yMeters

        xEffort = xError * Constants.DRIVE_TRANSLATION_KP
        yEffort = yError * Constants.DRIVE_TRANSLATION_KP

        direction = math.atan2(yEffort, xEffort)
        magnitude = math.sqrt(xError**2 + yError**2)

        if abs(magnitude) > Constants.DRIVE_MAX_SPEED_METERS_PER_SEC:
            magnitude = math.copysign(Constants.DRIVE_MAX_SPEED_METERS_PER_SEC, magnitude)

        self.applySpeeds(direction, magnitude, self.calcThetaControlRadPerSec(headingRad), True)

    def followLine(self, odomXMetersTarget: float, odomLineYMeters: float, lineSensorArray: LineSensorArray, headingRad = 0.0, findLineIfOff = True):
        driveEffort = (odomXMetersTarget - self.odometry.xMeters) * Constants.DRIVE_TRANSLATION_KP
        if abs(driveEffort) > Constants.DRIVE_FOLLOW_LINE_SPEED_METERS_PER_SEC:
            driveEffort = math.copysign(Constants.DRIVE_FOLLOW_LINE_SPEED_METERS_PER_SEC, driveEffort)
        
        if lineSensorArray.hasLine():
            self.odometry.yMeters = odomLineYMeters

            strafeEffort = lineSensorArray.getError() * Constants.FOLLOW_LINE_KP + lineSensorArray.getRate() * Constants.FOLLOW_LINE_KD

            self.applySpeedsCartesian(driveEffort, strafeEffort, self.calcThetaControlRadPerSec(headingRad), True)
        elif findLineIfOff:
            if lineSensorArray.getLastSensorOnLine() == RIGHT:
                self.applySpeedsCartesian(driveEffort, -Constants.DRIVE_FIND_LINE_SPEED_METERS_PER_SEC, self.calcThetaControlRadPerSec(headingRad), True)
            elif lineSensorArray.getLastSensorOnLine() == LEFT:
                self.applySpeedsCartesian(driveEffort, Constants.DRIVE_FIND_LINE_SPEED_METERS_PER_SEC, self.calcThetaControlRadPerSec(headingRad), True)
            else:
                self.stop()

timer = Timer()

drive = Drive()

frontLine = LineSensorArray(DevicePorts.FL_LINE, DevicePorts.FR_LINE)


def robotPeriodic():
    timer.event(robotPeriodic, Constants.LOOP_PERIOD_MS)

    xIn = -controller.axis4.position() * 0.01
    yIn = -controller.axis3.position() * 0.01
    thetaIn = -controller.axis1.position() * 0.01

    direction = math.atan2(yIn, xIn)
    magnitude = math.sqrt(xIn**2 + yIn**2)

    # drive.applySpeeds(direction, magnitude * 1.0, thetaIn * 2 * math.pi, True)

    drive.followLine(-1.25, 0, frontLine, findLineIfOff = True)

    print("X Meters: " + str(drive.odometry.xMeters), "Y Meters: " + str(drive.odometry.yMeters), "Rotation Radians: " + str(drive.odometry.thetaRad))
    # print("Line error: " + str(frontLine.getError()), frontLine.getRate())
    

    # print(drive.getDistanceTravelerodMeters())

robotPeriodic()
