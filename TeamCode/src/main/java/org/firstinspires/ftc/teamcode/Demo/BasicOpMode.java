package org.firstinspires.ftc.teamcode.Demo;

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

        // everything before this line happens on innit. everything after is on start.
        waitForStart();

        while (opModeIsActive()) {
            // powering a motor and accessing gamepads
            // motors accept a power between -1 and 1, where 1 is max power, -1, is max power in the opposite direction, and 0 is stopped
            motor.setPower(gamepad1.left_stick_y);

            // telemetry - sends data to driver hub
            telemetry.addData("Motor pos", motor.getCurrentPosition());
            telemetry.update();

            // servos are set to a position between 0 and 1
            // they automatically go to this position - no need to set power
            servo.setPosition(gamepad1.left_trigger);



        }
    }
}