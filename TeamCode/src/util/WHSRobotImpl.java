package util;

import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Odometry;

/**
 * Created by Jason on 10/20/2017.
 */

public class WHSRobotImpl {

    public util.Drivetrain drivetrain;
    public util.IMU imu;
    //public Odometry odometry;
    public OdometryWrapper odometryWrapper;

    util.Coordinate currentCoord;
    private double targetHeading; //field frame
    public double angleToTargetDebug;
    public double distanceToTargetDebug = 0;
    public util.Position vectorToTargetDebug = new util.Position(542, 542);
    private double lastKnownHeading = 0.1;

    private static double DEADBAND_DRIVE_TO_TARGET = util.RobotConstants.DEADBAND_DRIVE_TO_TARGET; //in mm
    private static double DEADBAND_ROTATE_TO_TARGET = util.RobotConstants.DEADBAND_ROTATE_TO_TARGET; //in degrees

    public static double DRIVE_MIN = util.RobotConstants.drive_min;
    public static double DRIVE_MAX = util.RobotConstants.drive_max;
    public static double ROTATE_MIN = util.RobotConstants.rotate_min;
    public static double ROTATE_MAX = util.RobotConstants.rotate_max;

    private static double ROTATE_KP = util.RobotConstants.R_KP;
    private static double ROTATE_KI = util.RobotConstants.R_KI;
    private static double ROTATE_KD = util.RobotConstants.R_KD;

    private static double DRIVE_KP = util.RobotConstants.D_KP;
    private static double DRIVE_KI = util.RobotConstants.D_KI;
    private static double DRIVE_KD = util.RobotConstants.D_KD;

    public util.PIDController rotateController = new util.PIDController(ROTATE_KP, ROTATE_KI, ROTATE_KD);
    public util.PIDController driveController = new util.PIDController(DRIVE_KP, DRIVE_KI, DRIVE_KD);

    private boolean firstRotateLoop = true;
    private boolean firstDriveLoop = true;
    private boolean driveBackwards;

    private int driveSwitch = 0;

    private boolean driveToTargetInProgress = false;
    private boolean rotateToTargetInProgress = false;

    private double[] encoderDeltas = {0.0, 0.0, 0.0};
    private double[] encoderValues = {0.0, 0.0};
    private int[] odometryEncoderDeltas = {0, 0, 0};
    private double robotX;
    private double robotY;
    private double distance;

    boolean firstRetractionLoop = true;
    util.SimpleTimer deadWheelPickupTimer = new util.SimpleTimer();
    double deadWheelPickupDelay = 0.5;

    public WHSRobotImpl(HardwareMap hardwareMap) {
        DEADBAND_DRIVE_TO_TARGET = util.RobotConstants.DEADBAND_DRIVE_TO_TARGET; //in mm
        DEADBAND_ROTATE_TO_TARGET = util.RobotConstants.DEADBAND_ROTATE_TO_TARGET; //in degrees

        DRIVE_MIN = util.RobotConstants.drive_min;
        DRIVE_MAX = util.RobotConstants.drive_max;
        ROTATE_MIN = util.RobotConstants.rotate_min;
        ROTATE_MAX = util.RobotConstants.rotate_max;

        ROTATE_KP = util.RobotConstants.R_KP;
        ROTATE_KI = util.RobotConstants.R_KI;
        ROTATE_KD = util.RobotConstants.R_KD;

        DRIVE_KP = util.RobotConstants.D_KP;
        DRIVE_KI = util.RobotConstants.D_KI;
        DRIVE_KD = util.RobotConstants.D_KD;

        drivetrain = new util.Drivetrain(hardwareMap);
        drivetrain.resetEncoders();
        imu = new util.IMU(hardwareMap);
        //odometry = hardwareMap.odometry.get("odometry");
        //odometryWrapper = new OdometryWrapper(hardwareMap);
        currentCoord = new util.Coordinate(0.0, 0.0, 0.0);
    }

    public WHSRobotImpl() {
    }

    public void driveToTarget(util.Position targetPos, boolean backwards) {
        util.Position vectorToTarget = util.Functions.Positions.subtract(targetPos, currentCoord.getPos()); //field frame
        vectorToTarget = util.Functions.field2body(vectorToTarget, currentCoord); //body frame
        vectorToTargetDebug = vectorToTarget;
        double distanceToTarget = vectorToTarget.getX()/*util.Functions.calculateMagnitude(vectorToTarget) * (vectorToTarget.getX() >= 0 ? 1 : -1)*/;
        distanceToTargetDebug = distanceToTarget;

        double degreesToRotate = Math.atan2(vectorToTarget.getY(), vectorToTarget.getX()); //from -pi to pi rad
        degreesToRotate = degreesToRotate * 180 / Math.PI;
        targetHeading = util.Functions.normalizeAngle(currentCoord.getHeading() + degreesToRotate); //-180 to 180 deg

        switch (driveSwitch) {
            case 0:
                driveToTargetInProgress = true;
                rotateToTarget(targetHeading, backwards);
                if (!rotateToTargetInProgress()) {
                    driveSwitch = 1;
                }
                break;
            case 1:

                if (firstDriveLoop) {
                    driveToTargetInProgress = true;
                    driveController.init(distanceToTarget);
                    firstDriveLoop = false;
                }

                driveController.setConstants(DRIVE_KP, DRIVE_KI, DRIVE_KD);
                driveController.calculate(distanceToTarget);

                double power = util.Functions.map(Math.abs(driveController.getOutput()), DEADBAND_DRIVE_TO_TARGET, 1500, DRIVE_MIN, DRIVE_MAX);

                // this stuff may be causing the robot to oscillate around the target position
                if (distanceToTarget < 0) {
                    power = -power;
                } else if (distanceToTarget > 0) {
                    power = Math.abs(power);
                }
                if (Math.abs(distanceToTarget) > DEADBAND_DRIVE_TO_TARGET) {
                    driveToTargetInProgress = true;
                    drivetrain.operateLeft(power);
                    drivetrain.operateRight(power);
                } else {
                    drivetrain.operateRight(0.0);
                    drivetrain.operateLeft(0.0);
                    driveToTargetInProgress = false;
                    rotateToTargetInProgress = false;
                    firstDriveLoop = true;
                    driveSwitch = 0;
                }
                // end of weird code
                break;
        }
    }

    public void rotateToTarget(double targetHeading, boolean backwards) {

        double angleToTarget = targetHeading - currentCoord.getHeading();
        /*if (backwards && angleToTarget > 90) {
            angleToTarget = angleToTarget - 180;
            driveBackwards = true;
        }
        else if (backwards && angleToTarget < -90) {
            angleToTarget = angleToTarget + 180;
            driveBackwards = true;
        }*/
        if (backwards) {
            angleToTarget = util.Functions.normalizeAngle(angleToTarget + 180); //-180 to 180 deg
            driveBackwards = true;
        } else {
            angleToTarget = util.Functions.normalizeAngle(angleToTarget);
            driveBackwards = false;
        }

        angleToTargetDebug = angleToTarget;

        if (firstRotateLoop) {
            rotateToTargetInProgress = true;
            rotateController.init(angleToTarget);
            firstRotateLoop = false;
        }

        rotateController.setConstants(ROTATE_KP, ROTATE_KI, ROTATE_KD);
        rotateController.calculate(angleToTarget);

        double power = (rotateController.getOutput() >= 0 ? 1 : -1) * (util.Functions.map(Math.abs(rotateController.getOutput()), 0, 180, ROTATE_MIN, ROTATE_MAX));

        if (Math.abs(angleToTarget) > DEADBAND_ROTATE_TO_TARGET/* && rotateController.getDerivative() < 40*/) {
            drivetrain.operateLeft(-power);
            drivetrain.operateRight(power);
            rotateToTargetInProgress = true;
        } else {
            drivetrain.operateLeft(0.0);
            drivetrain.operateRight(0.0);
            rotateToTargetInProgress = false;
            firstRotateLoop = true;
        }
    }

    public boolean driveToTargetInProgress() {
        return driveToTargetInProgress;
    }

    public boolean rotateToTargetInProgress() {
        return rotateToTargetInProgress;
    }

    public void estimatePosition() {
        encoderDeltas = drivetrain.getLRAvgEncoderDelta();
        distance = drivetrain.encToMM((encoderDeltas[0] + encoderDeltas[1]) / 2);
        robotX += distance * util.Functions.cosd(getCoordinate().getHeading());
        robotY += distance * util.Functions.sind(getCoordinate().getHeading());
        currentCoord.setX(robotX);
        currentCoord.setY(robotY);
    }

    /*    public void retractDeadwheelPickup(){
            if(firstRetractionLoop){
                deadWheelPickupTimer.set(deadWheelPickupDelay);
                firstRetractionLoop = false;
            }
            if (!deadWheelPickupTimer.isExpired()) {
                deadWheelPickup.setPosition(DeadWheelPickup.DeadWheelPickupPosition.UP);
                drivetrain.operate(-.2, -.2);
            }else{
                drivetrain.operate(0,0);
                deadwheelRetracted = true;
            }
        }*/
    public void deadWheelEstimateCoordinate() {
        //estimateHeading();
        //encoderDeltas = drivetrain.getAllEncoderDelta();
        //currentCoord.setHeading(/*util.Functions.normalizeAngle(Math.toDegrees(drivetrain.lrWheelConverter.encToMM(drivetrain.getAllEncoderPositions()[1]-drivetrain.getAllEncoderPositions()[0])/(util.Drivetrain.getTrackWidth())))*/ imu.getHeading());

        //For sim
        odometryEncoderDeltas = odometryWrapper.calculateOdometryDeltas();
        double deltaXWheels = (-odometryEncoderDeltas[0]/Odometry.ticksPerMM + odometryEncoderDeltas[1]/Odometry.ticksPerMM)/2;
        double deltaYWheel = odometryEncoderDeltas[2]/Odometry.ticksPerMM;
        double deltaTheta = ((odometryEncoderDeltas[1] - odometryEncoderDeltas[0])/Odometry.ticksPerMM) / (Odometry.robotOdometryRadius*2); // in radians

        /*double deltaXWheels = (drivetrain.lWheelConverter.encToMM(encoderDeltas[0]) + drivetrain.rWheelConverter.encToMM(encoderDeltas[1]))/2;
        double deltaYWheel = drivetrain.backWheelConverter.encToMM(encoderDeltas[2]);
        double deltaTheta = (drivetrain.rWheelConverter.encToMM(encoderDeltas[1]) - drivetrain.lWheelConverter.encToMM(encoderDeltas[0]))/(util.Drivetrain.getTrackWidth());*/
        /*double deltaXWheels = drivetrain.lrWheelConverter.encToMM((encoderDeltas[0] + encoderDeltas[1])/2);
        double deltaYWheel = drivetrain.backWheelConverter.encToMM(encoderDeltas[2]);
        double deltaTheta = drivetrain.lrWheelConverter.encToMM(encoderDeltas[1] - encoderDeltas[0])/(util.Drivetrain.getTrackWidth());
*/
        double movementRadius = deltaXWheels / (deltaTheta + .0001);
        double strafeRadius = deltaYWheel / (deltaTheta + .0001);

        double deltaXRobot = movementRadius * Math.sin(deltaTheta) + strafeRadius * (1 - Math.cos(deltaTheta));
        double deltaYRobot = strafeRadius * Math.sin(deltaTheta) - movementRadius * (1 - Math.cos(deltaTheta));

        util.Position bodyVector = new util.Position(deltaXRobot, deltaYRobot);
        util.Position fieldVector = util.Functions.body2field(bodyVector, currentCoord);
        currentCoord.addX(fieldVector.getX());
        currentCoord.addY(fieldVector.getY());
        currentCoord.setHeading(Functions.normalizeAngle(currentCoord.getHeading() + Math.toDegrees(deltaTheta)));
        //currentCoord.setPos(util.Functions.Positions.add(fieldVector, currentCoord));
    }

    /*public void deadWheelEstimateCoordinate() {
        encoderDeltas = drivetrain.getAllEncoderDelta();
        double leftDelta = drivetrain.lrWheelConverter.encToMM(encoderDeltas[0]);
        double rightDelta = drivetrain.lrWheelConverter.encToMM(encoderDeltas[1]);
        double backDelta = drivetrain.backWheelConverter.encToMM(encoderDeltas[2]);

        double currentHeading = currentCoord.getHeading();
        double angleDelta = currentHeading - lastKnownHeading;
        lastKnownHeading = currentHeading;
        double deltaX = 0.0;
        double deltaY = 0.0;

        double X = 0.0;
        if(Math.abs(rightDelta) > Math.abs(leftDelta)){
            X = leftDelta/(angleDelta+0.001);
            deltaY = util.Functions.sind(-angleDelta) * (X + drivetrain.L_DEAD_WHEEL_TO_ROBOT_CENTER);
            deltaX = (X + drivetrain.L_DEAD_WHEEL_TO_ROBOT_CENTER) - (util.Functions.cosd(angleDelta) * (X + drivetrain.L_DEAD_WHEEL_TO_ROBOT_CENTER));

        }
        else if(Math.abs(leftDelta) > Math.abs(rightDelta)){
            X = rightDelta/-(angleDelta+0.001);
            deltaY = util.Functions.sind(angleDelta) * (X + drivetrain.L_DEAD_WHEEL_TO_ROBOT_CENTER);
            deltaX = (util.Functions.cosd(angleDelta) * (X + drivetrain.L_DEAD_WHEEL_TO_ROBOT_CENTER)) - (X + drivetrain.L_DEAD_WHEEL_TO_ROBOT_CENTER);

        }
        else {
            deltaX = leftDelta;
        }

        double G = backDelta - (angleDelta*drivetrain.B_DEAD_WHEEL_TO_ROBOT_CENTER);
        double deltaY2 = util.Functions.sind(angleDelta)*G;
        double deltaX2 = util.Functions.cosd(angleDelta)*G;

        deltaX = deltaX + deltaX2;
        deltaY = deltaY + deltaY2;

        util.Position bodyVector = new util.Position(deltaX, deltaY);
        util.Position fieldVector = util.Functions.body2field(bodyVector, currentCoord);
        currentCoord.setPos(util.Functions.Positions.add(fieldVector, currentCoord));
    }*/

    public void mecanumEstimatePosition() {
//        encoderDeltas = drivetrain.getMecanumEncoderDelta();
//        double theta = Math.atan(encoderDeltas[1] / encoderDeltas[0]);

        double[] allEncoderDeltas = drivetrain.getAllEncoderDelta();
        double deltaXRobot = (Drivetrain.encToMM(allEncoderDeltas[2]) + Drivetrain.encToMM(allEncoderDeltas[3])) / 2;
        double deltaYRobot = (Drivetrain.encToMM(allEncoderDeltas[1]) - Drivetrain.encToMM(allEncoderDeltas[3])) / 2;

        Position fieldDelta = Functions.body2field(new Position(deltaXRobot, deltaYRobot), currentCoord);
        currentCoord.addX(fieldDelta.getX());
        currentCoord.addY(fieldDelta.getY());
    }

    public void estimateHeading() {
        double currentHeading;
        currentHeading = util.Functions.normalizeAngle(imu.getHeading() + imu.getImuBias()); //-180 to 180 deg
        currentCoord.setHeading(currentHeading); //updates global variable
    }

    public void setInitialCoordinate(util.Coordinate initCoord) {
        currentCoord = initCoord;
        robotX = initCoord.getX();
        robotY = initCoord.getY();
        imu.setImuBias(currentCoord.getHeading());
        lastKnownHeading = currentCoord.getHeading();
    }

    public void setCoordinate(util.Coordinate coord) {
        currentCoord = coord;
        imu.setImuBias(currentCoord.getHeading());
    }

    public util.Coordinate getCoordinate() {
        return currentCoord;
    }

    public void estimateCoordinate() {
        double[] currentEncoderValues = drivetrain.getLRAvgEncoderPosition();
        encoderDeltas[0] = currentEncoderValues[0] - encoderValues[0];
        encoderDeltas[1] = currentEncoderValues[1] - encoderValues[1];
        double currentHeading = util.Functions.normalizeAngle(Math.toDegrees(drivetrain.encToMM((currentEncoderValues[1] - currentEncoderValues[0]) / 2 / util.Drivetrain.getTrackWidth())) + imu.getImuBias()); //-180 to 180 deg
        currentCoord.setHeading(currentHeading); //updates global variable

        double deltaS = drivetrain.encToMM((encoderDeltas[0] + encoderDeltas[1]) / 2);
        double deltaHeading = Math.toDegrees(drivetrain.encToMM((encoderDeltas[1] - encoderDeltas[0]) / util.Drivetrain.getTrackWidth()));
        robotX += deltaS * util.Functions.cosd(lastKnownHeading + deltaHeading / 2);
        robotY += deltaS * util.Functions.sind(lastKnownHeading + deltaHeading / 2);

        currentCoord.setX(robotX);
        currentCoord.setY(robotY);
        encoderValues[0] = currentEncoderValues[0];
        encoderValues[1] = currentEncoderValues[1];
        lastKnownHeading = currentCoord.getHeading();
    }
}

