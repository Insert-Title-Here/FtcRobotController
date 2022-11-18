package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Common.MaintainLiftPosition;
import org.firstinspires.ftc.teamcode.Common.MecanumDrive;
import org.firstinspires.ftc.teamcode.Common.Vector2D;

@TeleOp
public class TeleOp2 extends LinearOpMode {

    MecanumDrive drive;
    DcMotor turret, lift;

    private final double NORMAL_LINEAR_MODIFIER = 0.6;
    private final double NORMAL_ROTATIONAL_MODIFIER = 0.6;
    private final double SPRINT_LINEAR_MODIFIER = 1;
    private final double SPRINT_ROTATIONAL_MODIFIER = 1;

    @Override
    public void runOpMode() throws InterruptedException {
        //drive = new MecanumDrive(hardwareMap, telemetry);
        turret = hardwareMap.get(DcMotor.class, "Turret");
        lift = hardwareMap.get(DcMotor.class, "SlideString");
        telemetry.addData("origTurretPos", turret.getCurrentPosition());
        telemetry.addData("origLiftPos", lift.getCurrentPosition());
        telemetry.update();

        waitForStart();
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        while (opModeIsActive()) {
            if (gamepad1.dpad_left&&turret.getCurrentPosition()>-200) {
                turret.setPower(-0.15);
                telemetry.addData("turretPos", turret.getCurrentPosition());
                telemetry.update();
            } else if (gamepad1.dpad_right&&turret.getCurrentPosition()<200) {
                turret.setPower(0.15);
                telemetry.addData("turretPos", turret.getCurrentPosition());
                telemetry.update();
            } else {
                turret.setPower(0);
            }

            if (gamepad1.left_trigger > 0.1) {
                lift.setPower(gamepad1.left_trigger * 0.8);
                telemetry.addData("liftPos", lift.getCurrentPosition());
                telemetry.update();

            } else if (gamepad1.right_trigger > 0.1) {
                lift.setPower(-gamepad1.right_trigger * 0.6);
                telemetry.addData("liftPos", lift.getCurrentPosition());
                telemetry.update();

            }


        }

    }
}