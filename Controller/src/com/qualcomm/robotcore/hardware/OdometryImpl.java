package com.qualcomm.robotcore.hardware;

public class OdometryImpl implements Odometry {

    // left, right, center
    private int[] encoderCounts = {0, 0, 0};
    private int[] encoderDeltas = {0, 0, 0};

    public synchronized void update(double deltaRX, double deltaRY, double deltaRTheta) {
        // +x: The right side of the robot. +y: the front of the robot
        encoderDeltas[0] = (int) ((deltaRY - deltaRTheta * robotOdometryRadius) * ticksPerMM);
        encoderDeltas[1] = (int) ((deltaRY + deltaRTheta * robotOdometryRadius) * ticksPerMM);
        encoderDeltas[2] = (int) ((deltaRX + deltaRTheta * robotOdometryRadius) * ticksPerMM);
        encoderCounts[0] += encoderDeltas[0];
        encoderCounts[1] += encoderDeltas[1];
        encoderCounts[2] += encoderDeltas[2];
    }

    public synchronized int getLWheelCount() {
        return encoderCounts[0];
    }

    public synchronized int getRWheelCount() {
        return encoderCounts[1];
    }

    public synchronized int getCWheelCount() {
        return encoderCounts[2];
    }

    public synchronized int[] getAllWheelCounts() {
        return encoderCounts;
    }

    public synchronized void reset() {
        encoderCounts = new int[] {0, 0, 0};
        encoderDeltas = new int[] {0, 0, 0};
    }

}
