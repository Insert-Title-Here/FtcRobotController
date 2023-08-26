package org.firstinspires.ftc.teamcode.Demo.DriveTrainExamples;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "BasicOpModeOOP")
public class BasicOpModeOOP extends LinearOpMode {

    DcMotor motor;
    Servo servo;
    DriveTrainExample drive;

    public final double LINEAR_MODIFIER = 1;
    public final double ROTATIONAL_MODIFIER = 0.7;

    public void runOpMode() throws InterruptedException {
        // deviceName is what its called in config

        drive = new DriveTrainExample(hardwareMap);

        waitForStart();

        while (opModeIsActive()) {

            drive.setPower(gamepad1.left_stick_y * LINEAR_MODIFIER, gamepad1.right_stick_x * ROTATIONAL_MODIFIER);

        }
    }
}