package org.firstinspires.ftc.teamcode.ftc16072;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import util.*;

;


@TeleOp(name = "Swerve to target test", group = "Mechanum")
public class SwerveToTargetTest extends OpMode {
    WHSRobotImpl robot;
    Robot robotToo;
    Coordinate startingCoordinate = new Coordinate(0, 0, 0);
    static Position p1 = new Position(0, 0);
    static Position p2 = new Position(1200,1200);
    static Position p3 = new Position(0, 2438.4);/*
    static Position p4 = new Position(-1800, 2700);
    static Position p5 = new Position(-1800, 0);*/

    Position[] positions1 = {startingCoordinate.getPos(), p2, p3};
    SwerveToTarget swerve1;
    SwerveToTarget swerve2;
    int state = 1;
    double[] swervePowers;


    @Override
    public void init() {
        robot = new WHSRobotImpl(hardwareMap);
        robot.drivetrain.resetEncoders();
        robot.setInitialCoordinate(startingCoordinate);
        swerve1 = new SwerveToTarget(SwerveConstants.kP, SwerveConstants.kV, SwerveConstants.kA, positions1, 10, 0.99, 3, SwerveConstants.lookaheadDistance, 1000);
    }

    @Override
    public void loop() {
        robot.estimateHeading();
        robot.estimatePosition();
        swervePowers = swerve1.calculateMotorPowers(robot.getCoordinate(), false);

        robot.drivetrain.operate(swervePowers[0], swervePowers[1]);

        telemetry.addData("x", robot.getCoordinate().getX());
        telemetry.addData("y", robot.getCoordinate().getY());
        telemetry.addData("heading", robot.getCoordinate().getHeading());
        telemetry.addData("index of last closest point", swerve1.lastClosestPointIndex);
        telemetry.addData("left power", swervePowers[0]);
        telemetry.addData("right power", swervePowers[1]);
        for (int i = 0; i < swerve1.smoothedPath.length; i++) {
            telemetry.addData("point" + i, swerve1.smoothedPath[i].getX() + ", " + swerve1.smoothedPath[i].getY());
        }
        for (int i = 0; i < swerve1.targetVelocities.length; i++) {
            telemetry.addData("velocity" + i, swerve1.targetVelocities[i]);
        }
        telemetry.addData("lookahead point", swerve1.lookaheadPoint.getX() + ", " + swerve1.lookaheadPoint.getY());
        telemetry.addData("Target Left Velcoities", swerve1.getCurrentTargetWheelVelocities()[0]);
        telemetry.addData("Target Right Velcoities ", swerve1.getCurrentTargetWheelVelocities()[1]);
/*        telemetry.addData("Current Velocities Left", robot.drivetrain.getAllWheelVelocities()[0]);
        telemetry.addData("Current Velocities Right", robot.drivetrain.getAllWheelVelocities()[1]);*/
    }
}
