package org.firstinspires.ftc.teamcode.ftc16072;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import util.*;

@Autonomous(name = "DriveToTarget Test")
public class DriveToTargetTest extends OpMode {
    WHSRobotImpl robot;

    Coordinate startingCoordinate = new Coordinate(0, 0, 0);
    Position p1 = new Position(600, -600);
    Position p2 = new Position(600,500);
    int state = 0;


    @Override
    public void init() {
        robot = new WHSRobotImpl(hardwareMap);
        robot.setInitialCoordinate(new Coordinate(0, 0, 0));
    }

    @Override
    public void loop() {
        robot.estimatePosition();
        robot.estimateHeading();
        switch(state){
            case 0:
                robot.setInitialCoordinate(startingCoordinate);
                state++;
                break;
            case 1:
                robot.driveToTarget(p1, true);
                if (!robot.driveToTargetInProgress() && !robot.rotateToTargetInProgress()) {
                    state++;
                }
                break;
            case 2:
                robot.driveToTarget(p2, false);
                if (!robot.driveToTargetInProgress() && !robot.rotateToTargetInProgress()) {
                    state++;
                }
                break;
            default:
                break;
        }

        telemetry.addData("DriveToTarget in progress: ", robot.driveToTargetInProgress());
        telemetry.addData("RotateToTarget in progress: ", robot.rotateToTargetInProgress());
        telemetry.addData("IMU", robot.imu.getHeading());
        telemetry.addData("X", robot.getCoordinate().getX());
        telemetry.addData("Y", robot.getCoordinate().getY());
        telemetry.addData("Heading", robot.getCoordinate().getHeading());
        telemetry.addData("FL Power", robot.drivetrain.frontLeft.getPower());
        telemetry.addData("BL Power", robot.drivetrain.backLeft.getPower());
        telemetry.addData("FR Power", robot.drivetrain.frontRight.getPower());
        telemetry.addData("BR Power", robot.drivetrain.backRight.getPower());
        telemetry.addData("Distance to target", robot.distanceToTargetDebug);
        telemetry.addData("Angle to target", robot.angleToTargetDebug);
        telemetry.addData("BLdelta", robot.drivetrain.backLeft.getCurrentPosition());
        telemetry.addData("BRdelta", robot.drivetrain.backRight.getCurrentPosition());
        telemetry.addData("FLdelta", robot.drivetrain.frontLeft.getCurrentPosition());
        telemetry.addData("FRdelta", robot.drivetrain.frontRight.getCurrentPosition());
        telemetry.addData("Rotate Integral", robot.rotateController.getIntegral());
        telemetry.addData("Rotate Derivative", robot.rotateController.getDerivative());
        telemetry.addData("Drive Integral", robot.driveController.getIntegral());
        telemetry.addData("Drive Derivative", robot.driveController.getDerivative());
        telemetry.addData("Vector to Target x", robot.vectorToTargetDebug.getX());
        telemetry.addData("Vector to Target y", robot.vectorToTargetDebug.getY());

    }
}
