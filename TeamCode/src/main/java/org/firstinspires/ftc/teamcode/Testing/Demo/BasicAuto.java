package org.firstinspires.ftc.teamcode.Testing.Demo;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "BasicTeleop")
public class BasicAuto extends LinearOpMode {

    DriveTrainYay drive;

    double LINEAR_MODIFIER = 0.5;
    double ROTATIONAL_MODIFIER = 0.3;
    public void runOpMode() throws InterruptedException {

        drive = new DriveTrainYay(hardwareMap);

        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.right_bumper) {
                drive.setPower(gamepad1.left_stick_y, gamepad1.right_stick_x);
            } else {
                drive.setPower(gamepad1.left_stick_y * LINEAR_MODIFIER, gamepad1.right_stick_x * ROTATIONAL_MODIFIER);

            }
            telemetry.update();
        }


    }
}