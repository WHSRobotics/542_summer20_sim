package org.firstinspires.ftc.teamcode.ftc16072;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import util.WHSRobotImpl;
import virtual_robot.controller.robots.classes.MechanumBot;

@Autonomous(name = "Drivetrain Test")
public class DrivetrainTest extends OpMode {

    WHSRobotImpl robot;
    MechanumBot mechanumBot = new MechanumBot();
    double prevTime;
    double currentTime;

    @Override
    public void init() {
        robot = new WHSRobotImpl(hardwareMap);

    }

    @Override
    public void start() {
        currentTime = System.currentTimeMillis();
        prevTime = currentTime;
    }

    @Override
    public void loop() {
        robot.deadWheelEstimateCoordinate();
        robot.drivetrain.operateMecanumDrive(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x, robot.getCoordinate().getHeading());
        //robot.drivetrain.operate(-gamepad1.left_stick_y, -gamepad1.right_stick_y);
//        robot.simEstimateCoordinate();


//        telemetry.addData("Left Distance", robot.simGetDistances()[0]);
//        telemetry.addData("Right Distance", robot.simGetDistances()[1]);
//        telemetry.addData("Front Distance", robot.simGetDistances()[2]);
//        telemetry.addData("Back Distance", robot.simGetDistances()[3]);
        telemetry.addData("X", robot.getCoordinate().getX());
        telemetry.addData("Y", robot.getCoordinate().getY());
        telemetry.addData("Heading", robot.getCoordinate().getHeading());
    }
}
