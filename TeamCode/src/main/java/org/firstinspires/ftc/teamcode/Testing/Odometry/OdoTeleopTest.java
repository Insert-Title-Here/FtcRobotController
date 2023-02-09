package org.firstinspires.ftc.teamcode.Testing.Odometry;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Competition.Interleagues.Common.Vector2D;

public class OdoTeleopTest extends LinearOpMode {

    MecDriveSimple drive;

    private final double NORMAL_LINEAR_MODIFIER = 0.6;
    private final double NORMAL_ROTATIONAL_MODIFIER = 0.6;
    private final double SPRINT_LINEAR_MODIFIER = 1;
    private final double SPRINT_ROTATIONAL_MODIFIER = 1;

    @Override
    public void runOpMode() throws InterruptedException {

        drive = new MecDriveSimple(hardwareMap, telemetry);


        waitForStart();


        while (opModeIsActive()) {

            if (gamepad1.right_bumper) {
                drive.setPower(new Vector2D(gamepad1.right_stick_x * SPRINT_LINEAR_MODIFIER, gamepad1.left_stick_y * SPRINT_LINEAR_MODIFIER), gamepad1.left_stick_x * SPRINT_ROTATIONAL_MODIFIER, false);
            } else {
                drive.setPower(new Vector2D(gamepad1.right_stick_x * NORMAL_LINEAR_MODIFIER, gamepad1.left_stick_y * NORMAL_LINEAR_MODIFIER), gamepad1.left_stick_x * NORMAL_ROTATIONAL_MODIFIER, false);
            }

            telemetry.addData("pod 1 - FL", drive.getFLPosition());
            telemetry.addData("pod 2 - FR", drive.getFRPosition());
            telemetry.addData("pod 3 - BL", drive.getBLPosition());
            telemetry.update();

        }
    }

}