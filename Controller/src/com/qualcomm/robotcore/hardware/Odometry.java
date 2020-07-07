package com.qualcomm.robotcore.hardware;

public interface Odometry extends HardwareDevice {

    double ticksPerMM = 100.0;
    //Distance all wheels are from the center of the robot
    double robotOdometryRadius = 200.0;

    /**
     * Updates the state of odometry
     * @param deltaRX Change in robot x, in mm
     * @param deltaRY Change in robot y, in mm
     * @param deltaRTheta Change in robot heading, in radians
     */
    void update(double deltaRX, double deltaRY, double deltaRTheta);

    int getLWheelCount();
    int getRWheelCount();
    int getCWheelCount();
    int[] getAllWheelCounts();

    int getLWheelDelta();
    int getRWheelDelta();
    int getCWheelDelta();

    int[] getAllWheelDeltas();

}
