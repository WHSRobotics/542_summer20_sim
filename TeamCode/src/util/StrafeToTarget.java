package util;

public class StrafeToTarget {

    private util.Coordinate currentCoord;

    private static final double MAXIMUM_ACCELERATION = util.SwerveConstants.MAX_ACCELERATION; // mm/s^2
    private static final double MAXIMUM_VELOCITY = util.SwerveConstants.MAX_VELOCITY;
    private static final double MAXIMUM_ANGULAR_ACCELERATION = util.SwerveConstants.MAX_ANGULAR_ACCELERATION;
    private static final double MAXIMUM_ANGULAR_VELOCITY = SwerveConstants.MAX_ANGULAR_VELOCITY;
    private double pathMaximumVelocity;
    public int lastClosestPointIndex = 0;
    public int lastClosestHeadingIndex = 0;
    private int lastIndex = 0;
    private double currentTValue = 0;
    private static final double hKI = util.SwerveConstants.hKI;
    private static final double hKP = util.SwerveConstants.hKP;
    private static final double hKD = util.SwerveConstants.hKD;

    PIDController headingController = new PIDController(hKP, hKI, hKD);

    public util.Coordinate[] smoothedPath;
    private double[] targetCurvatures;
    public double[] targetVelocities;
    public double[] targetAngularVelocities;

    public boolean conditionMet = false;

    private double[] currentTargetWheelVelocities = {0.0, 0.0, 0.0, 0.0};
    private double[] lastTargetWheelVelocities = {0.0, 0.0, 0.0, 0.0};
    private double lastTime;
    private RateLimiter targetTangentialVelocityRateLimiter = new RateLimiter(MAXIMUM_ACCELERATION, 0);
    private RateLimiter targetAngularVelocityRateLimiter = new RateLimiter(MAXIMUM_ANGULAR_ACCELERATION, 0);
    private double kP;
    private double kV;
    private double kA;
    private double lookaheadDistance;
    private double trackWidth;
    private double wheelBase;
    public Position lookaheadPoint;
    private boolean inProgress;
    double angularVelocity;

    public double angleToLookaheadPointDebug = 0.0;

    public StrafeToTarget(double kP, double kV, double kA, util.Coordinate[] targetPositions, double spacing, double weightSmooth, double tolerance, double velocityConstant, double lookaheadDistance, double pathMaximumVelocity) {
        this.pathMaximumVelocity = pathMaximumVelocity;

        this.kP = kP;
        this.kV = 1 / MAXIMUM_VELOCITY;
        this.kA = kA;
        this.angularVelocity = angularVelocity;
        this.lookaheadDistance = lookaheadDistance;
        trackWidth = util.Drivetrain.getTrackWidth();
        wheelBase = util.Drivetrain.getWheelBase();
        OldPathGenerator oldPathGenerator = new OldPathGenerator();
        smoothedPath = oldPathGenerator.generateCoordPath(targetPositions, spacing, weightSmooth);
        targetVelocities = oldPathGenerator.calculateTargetVelocities(velocityConstant, pathMaximumVelocity, MAXIMUM_ACCELERATION);
        //targetAngularVelocities = pathGenerator.calculateTargetAngularVelocities(MAXIMUM_ANGULAR_ACCELERATION, MAXIMUM_ANGULAR_VELOCITY);
        targetAngularVelocities = new double[smoothedPath.length];
        for (int i = smoothedPath.length-2; i >= 0; i--){
            double deltaTheta = smoothedPath[i+1].getHeading() - smoothedPath[i].getHeading();
            double sign = 1;
            if (deltaTheta < 0){
                sign = -1;
            }
            targetAngularVelocities[i] = sign * (Math.min(MAXIMUM_ANGULAR_VELOCITY, Math.sqrt((targetAngularVelocities[i+1] * targetAngularVelocities[i+1]) + 2 * MAXIMUM_ANGULAR_ACCELERATION * Math.abs(deltaTheta))));
        }
        lastTime = System.nanoTime() / 1E9;
    }

    public double[] calculateMotorPowers(util.Coordinate currentCoord) {
        this.currentCoord = currentCoord;
/*
        double[] currentWheelVelocities = {currentBackVelocities[1] - (frontRightVelocity - currentBackVelocities[0]), frontRightVelocity, currentBackVelocities[0], currentBackVelocities[1]};
*/

        boolean tFound = false;
        for (int i = lastIndex; i < smoothedPath.length - 1; i++) {
            Double nextTValue = new Double(calculateT(smoothedPath[i], smoothedPath[i + 1], lookaheadDistance));

            if (!tFound && !nextTValue.isNaN() && (nextTValue + i) > (currentTValue + lastIndex)) {
                tFound = true;
                currentTValue = nextTValue;
                lastIndex = i;
            }
        }

        Position calculatedTStartPoint = smoothedPath[lastIndex];
        Position calculatedTEndPoint = smoothedPath[lastIndex + 1];
        lookaheadPoint = util.Functions.Positions.add(calculatedTStartPoint, util.Functions.Positions.scale(currentTValue, util.Functions.Positions.subtract(calculatedTEndPoint, calculatedTStartPoint)));

        int indexOfClosestPoint = calculateIndexOfClosestPoint(smoothedPath);
        //int indexOfClosestHeading = calculateIndexOfClosestHeading();

        Position vectorToLookaheadPoint = util.Functions.Positions.subtract(lookaheadPoint, currentCoord);
        vectorToLookaheadPoint = util.Functions.field2body(vectorToLookaheadPoint, currentCoord);
        double angleToLookaheadPoint = Math.toDegrees(Math.atan2(vectorToLookaheadPoint.getY(), vectorToLookaheadPoint.getX()));
        angleToLookaheadPointDebug = angleToLookaheadPoint;

        headingController.calculate(smoothedPath[indexOfClosestPoint].getHeading() - currentCoord.getHeading());//(targetAngularVelocities[indexOfClosestHeading] - angularVelocity);
        double headingFeedback = headingController.getOutput();

        currentTargetWheelVelocities = calculateTargetWheelVelocities(targetVelocities[indexOfClosestPoint], angleToLookaheadPoint, targetAngularVelocities[indexOfClosestPoint] + headingFeedback);

        double deltaTime = System.nanoTime() / 1E9 - lastTime;
        double[] targetWheelAccelerations = new double[4];
        for (int i = 0; i < targetWheelAccelerations.length; i++) {
            targetWheelAccelerations[i] = (currentTargetWheelVelocities[i] - lastTargetWheelVelocities[i]) / deltaTime;
        }
        if (indexOfClosestPoint != smoothedPath.length - 1) {
            double[] feedBack = {0.0,0.0,0.0,0.0} /*{currentTargetWheelVelocities[0] - currentWheelVelocities[0], currentTargetWheelVelocities[1] - currentWheelVelocities[1], currentTargetWheelVelocities[2] - currentWheelVelocities[2], currentTargetWheelVelocities[3] - currentWheelVelocities[3]}*/;
            for (int i = 0; i < feedBack.length; i++) {
                feedBack[i] *= kP;
            }

            double[] feedForwardVel = {kV * currentTargetWheelVelocities[0], kV * currentTargetWheelVelocities[1], kV * currentTargetWheelVelocities[2], kV * currentTargetWheelVelocities[3]};
            double[] feedForwardAccel = {kA * targetWheelAccelerations[0], kA * targetWheelAccelerations[1], kA * targetWheelAccelerations[2], kA * targetWheelAccelerations[3]};
            double[] feedForward = {feedForwardVel[0] + feedForwardAccel[0], feedForwardVel[1] + feedForwardAccel[1], feedForwardVel[2] + feedForwardAccel[2], feedForwardVel[3] + feedForwardAccel[3]};
            double[] motorPowers = {util.Functions.constrain(feedBack[0] + feedForward[0]/* - headingFeedback*/, -1, 1), util.Functions.constrain(feedBack[1] + feedForward[1]/* + headingFeedback*/, -1, 1), util.Functions.constrain(feedBack[2] + feedForward[2]/* - headingFeedback*/, -1, 1), util.Functions.constrain(feedBack[3] + feedForward[3]/* + headingFeedback*/, -1, 1)};
            lastTargetWheelVelocities = currentTargetWheelVelocities;
            inProgress = true;
            return motorPowers;
        } else {
            inProgress = false;
        }

        return new double[] {0.0, 0.0, 0.0, 0.0};
    }

    private double calculateT(Position lineStart, Position lineEnd, double lookaheadDistance) {
        // constants used throughout the method
        Position d = util.Functions.Positions.subtract(lineEnd, lineStart);
        Position f = util.Functions.Positions.subtract(lineStart, currentCoord);
        double r = lookaheadDistance;

        double a = util.Functions.Positions.dot(d, d);
        double b = 2 * util.Functions.Positions.dot(f, d);
        double c = util.Functions.Positions.dot(f, f) - r * r;

        double discriminant = b * b - 4 * a * c;
        if (discriminant < 0) {
            // no intersection
        } else {
            // ray didn't totally miss sphere, so there is a solution to the equation.
            discriminant = Math.sqrt(discriminant);

            // either solution may be on or off the ray so need to test both
            // t1 is always the smaller value, because BOTH discriminant and a are nonnegative.
            double t1 = (-b - discriminant) / (2 * a);
            double t2 = (-b + discriminant) / (2 * a);

            // 3x HIT cases:
            //          -o->             --|-->  |            |  --|->
            // Impale(t1 hit,t2 hit), Poke(t1 hit,t2>1), ExitWound(t1<0, t2 hit),

            // 3x MISS cases:
            //       ->  o                     o ->              | -> |
            // FallShort (t1>1,t2>1), Past (t1<0,t2<0), CompletelyInside(t1<0, t2>1)

            if (t1 >= 0 && t1 <= 1) {
                // t1 is the intersection, and it's closer than t2 (since t1 uses -b - discriminant)
                // Impale, Poke
                return t1;
            }

            if (t2 >= 0 && t2 <= 1) {
                return t2;
            }
        }
        return Double.NaN;
    }

    private int calculateIndexOfClosestPoint(Position[] smoothedPath) {
        // creates array in which we store the current distance to each point in our path
        double[] distances = new double[smoothedPath.length];
        for (int i = 0/*lastClosestPointIndex*/; i < smoothedPath.length; i++) {
            distances[i] = util.Functions.Positions.subtract(smoothedPath[i], currentCoord).getMagnitude();
        }

        // calculates the index of value in the array with the smallest value and returns that index
        lastClosestPointIndex = util.Functions.calculateIndexOfSmallestValue(distances);
        return lastClosestPointIndex;
    }

    private int calculateIndexOfClosestHeading() {
        boolean closestHeadingFound = false;
        double currentHeading = currentCoord.getHeading();
        double[] headingDiffs = new double[smoothedPath.length];
        for (int i = lastClosestHeadingIndex; i < smoothedPath.length-1; i++) {
            if (!closestHeadingFound && Math.abs(currentHeading - smoothedPath[i+1].getHeading()) < Math.abs(currentHeading - smoothedPath[lastClosestHeadingIndex].getHeading())) {
                if (i < smoothedPath.length - 2) {
                    if (Math.abs(smoothedPath[i + 2].getHeading() - currentHeading) >= Math.abs(smoothedPath[i+1].getHeading() - currentHeading)) {
                        conditionMet = true;
                        lastClosestHeadingIndex = i + 1;
                        closestHeadingFound = true;
                    }
                } else {
                    closestHeadingFound = true;
                }
            }
            //headingDiffs[i] = Math.abs(currentHeading - smoothedPath[i].getHeading());
        }
        //lastClosestHeadingIndex = util.Functions.calculateIndexOfSmallestValue(headingDiffs);
        return lastClosestHeadingIndex;
    }

    public double[] calculateTargetTranslationalWheelVelocities(double targetVelocity, double angleToLookaheadPoint) {
        double targetVelocityX = targetVelocity * util.Functions.cosd(angleToLookaheadPoint);
        double targetVelocityY = targetVelocity * util.Functions.sind(angleToLookaheadPoint);
        double k = (trackWidth + wheelBase) / 2;

        double vFL = targetVelocityX - targetVelocityY;// - k * angleToLookaheadPoint;
        double vFR = targetVelocityX + targetVelocityY;// + k * angleToLookaheadPoint;
        double vBL = targetVelocityX + targetVelocityY;// - k * angleToLookaheadPoint;
        double vBR = targetVelocityX - targetVelocityY;// + k * angleToLookaheadPoint;

        return new double[]{vFL, vFR, vBL, vBR};
    }

    public double[] calculateTargetWheelVelocities(double targetVelocity, double angleToLookaheadPoint, double targetAngularVelocity) {
        double rateLimitedTargetTangentialVelocity = targetTangentialVelocityRateLimiter.calculateOutput(targetVelocity);
        double rateLimitedTargetAngularVelocity = Math.toRadians(targetAngularVelocityRateLimiter.calculateOutput(targetAngularVelocity));
        double targetVelocityX = rateLimitedTargetTangentialVelocity * util.Functions.cosd(angleToLookaheadPoint);
        double targetVelocityY = rateLimitedTargetTangentialVelocity * util.Functions.sind(angleToLookaheadPoint);
        double k = (trackWidth + wheelBase) / 2;

        double vFL = targetVelocityX - targetVelocityY - k * rateLimitedTargetAngularVelocity;
        double vFR = targetVelocityX + targetVelocityY + k * rateLimitedTargetAngularVelocity;
        double vBL = targetVelocityX + targetVelocityY - k * rateLimitedTargetAngularVelocity;
        double vBR = targetVelocityX - targetVelocityY + k * rateLimitedTargetAngularVelocity;

        return new double[]{vFL, vFR, vBL, vBR};
    }


}