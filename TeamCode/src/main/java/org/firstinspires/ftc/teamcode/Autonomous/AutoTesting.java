package org.firstinspires.ftc.teamcode.Autonomous;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Common.MecanumDrive;
import org.firstinspires.ftc.teamcode.Common.Vector2D;

public class AutoTesting extends LinearOpMode {

    MecanumDrive drive;

    //TODO: Move scoring system stuff to its own class
    DcMotor lift;
    Servo clawl;
    Servo clawr;


    private final double NORMAL_LINEAR_MODIFIER = 0.6;
    private final double NORMAL_ROTATIONAL_MODIFIER = 0.6;
    private final double SPRINT_LINEAR_MODIFIER = 1;
    private final double SPRINT_ROTATIONAL_MODIFIER = 1;

    @Override
    public void runOpMode() throws InterruptedException {

        drive = new MecanumDrive(hardwareMap, telemetry);
        lift = hardwareMap.get(DcMotor.class, "lift");
        clawl = hardwareMap.get(Servo.class, "clawl");
        clawr = hardwareMap.get(Servo.class, "clawr");

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        clawl.setPosition(0);
        clawr.setPosition(0.6);

        waitForStart();


        while (opModeIsActive()) {

            if (gamepad1.right_bumper) {
                drive.setPower(new Vector2D(gamepad1.right_stick_x * SPRINT_LINEAR_MODIFIER, gamepad1.left_stick_y * SPRINT_LINEAR_MODIFIER), gamepad1.left_stick_x * SPRINT_ROTATIONAL_MODIFIER, false);
            } else {
                drive.setPower(new Vector2D(gamepad1.right_stick_x * NORMAL_LINEAR_MODIFIER, gamepad1.left_stick_y * NORMAL_LINEAR_MODIFIER), gamepad1.left_stick_x * NORMAL_ROTATIONAL_MODIFIER, false);
            }

            if (gamepad1.left_trigger > 0.1) {
                lift.setPower(gamepad1.left_trigger / 2);
            } else if (gamepad1.right_trigger > 0.1) {
                lift.setPower(-gamepad1.right_trigger / 2);
            } else {
                lift.setPower(0);
            }

            if (gamepad1.left_bumper) {
                clawl.setPosition(0);
                clawr.setPosition(0.6);
            }

            if (gamepad1.right_bumper) {
                clawr.setPosition(0);
                clawl.setPosition(0.6);
            }


            telemetry.addData("flPos", drive.getFLPosition());
            telemetry.addData("frPos", drive.getFRPosition());
            telemetry.addData("blPos", drive.getBLPosition());
            telemetry.addData("brPos", drive.getBRPosition());
            telemetry.update();


        }

        drive.setPower(0, 0, 0, 0);
    }
}
