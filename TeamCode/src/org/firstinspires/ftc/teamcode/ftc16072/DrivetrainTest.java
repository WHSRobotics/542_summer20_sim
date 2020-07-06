package org.firstinspires.ftc.teamcode.ftc16072;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import util.Drivetrain;

@Autonomous(name = "Drivetrain Test")
public class DrivetrainTest extends OpMode {

    Drivetrain drivetrain;

    @Override
    public void init() {
        drivetrain = new Drivetrain(hardwareMap);
    }

    @Override
    public void loop() {
        drivetrain.operate(gamepad1.left_stick_y, gamepad1.right_stick_y);
    }
}
