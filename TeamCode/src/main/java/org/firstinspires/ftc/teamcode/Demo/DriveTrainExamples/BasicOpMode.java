package org.firstinspires.ftc.teamcode.Demo.DriveTrainExamples;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "BasicOpMode")
public class BasicOpMode extends LinearOpMode {

    DcMotor motor;
    Servo servo;

    public void runOpMode() throws InterruptedException {
        // deviceName is what its called in config

        // initializing motors
        motor = hardwareMap.get(DcMotor.class, "motor");

        // servos
        servo = hardwareMap.get(Servo.class, "servo");

        waitForStart();

        while (opModeIsActive()) {

            motor.setPower(gamepad1.left_stick_y);
            telemetry.addData("Motor pos", motor.getCurrentPosition());

        }
    }
}