package org.firstinspires.ftc.teamcode.ftc16072;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import javafx.geometry.Pos;
import util.*;
import util.swervetotarget.FollowerConstants;
import util.swervetotarget.PathGenerator;
import util.swervetotarget.SwervePath;
import util.swervetotarget.SwervePathGenerationConstants;

;import java.nio.file.Path;
import java.util.ArrayList;
import java.util.Arrays;


@TeleOp(name = "Swerve to target test", group = "Mechanum")
public class SwerveToTargetTest extends OpMode {
    WHSRobotImpl robot;
    Coordinate startingCoordinate = new Coordinate(0, 0, 0);
    static Position p1 = new Position(0, 0);
    static Position p2 = new Position(1200,1200);
    static Position p3 = new Position(0, 2438.4);/*
    static Position p4 = new Position(-1800, 2700);
    static Position p5 = new Position(-1800, 0);*/
    ArrayList<Position> positions1 = new ArrayList<Position>();//(ArrayList<Position>) Arrays.asList(positions);// )
    FollowerConstants followerConstants = new FollowerConstants(600,false);
    SwervePathGenerationConstants swervePathGenerationConstants= new SwervePathGenerationConstants(10, 0.99, 3,1000);
    SwervePath path;


    @Override
    public void init() {
        robot = new WHSRobotImpl(hardwareMap);
        robot.drivetrain.resetEncoders();
        robot.setInitialCoordinate(startingCoordinate);
        positions1.add(startingCoordinate.getPos());
        positions1.add(p2);
        positions1.add(p3);
        path = PathGenerator.generateSwervePath(positions1, followerConstants,swervePathGenerationConstants);
    }
    @Override
    public void start(){
        robot.updatePath(path);
    }
    @Override
    public void loop() {
        robot.estimateHeading();
        robot.estimatePosition();
        robot.swerveToTarget();
        telemetry.addData("x", robot.getCoordinate().getX());
        telemetry.addData("y", robot.getCoordinate().getY());
        telemetry.addData("heading", robot.getCoordinate().getHeading());

/*        telemetry.addData("index of last closest point", swerve1.lastClosestPointIndex);
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
*//*        telemetry.addData("Current Velocities Left", robot.drivetrain.getAllWheelVelocities()[0]);
        telemetry.addData("Current Velocities Right", robot.drivetrain.getAllWheelVelocities()[1]);*/
    }
}
