package org.firstinspires.ftc.teamcode.ftc16072;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import util.Drivetrain;
import util.OdometryWrapper;

@TeleOp(name = "OdometryWrapperTest")
public class OdometryWrapperTest extends OpMode {

    Drivetrain drivetrain;
    OdometryWrapper odometryWrapper;
    int[] encoderDeltas = {0, 0, 0};

    @Override
    public void init() {
        drivetrain = new Drivetrain(hardwareMap);
        odometryWrapper = new OdometryWrapper(hardwareMap);
    }

    @Override
    public void loop() {
        encoderDeltas = odometryWrapper.calculateOdometryDeltas();
        telemetry.addData("L", encoderDeltas[0]);
        telemetry.addData("R", encoderDeltas[1]);
        telemetry.addData("C", encoderDeltas[2]);

    }
}
