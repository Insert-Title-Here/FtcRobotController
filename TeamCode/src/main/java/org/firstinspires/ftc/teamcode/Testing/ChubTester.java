package org.firstinspires.ftc.teamcode.Testing;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Competition.Interleagues.Common.Constants;

@TeleOp (name = "Testing CHUBS")
public class ChubTester extends LinearOpMode {

    DcMotor motor;
    Servo servo;

    @Override
    public void runOpMode() throws InterruptedException {

        servo = hardwareMap.get(Servo.class, "servo");
        motor = hardwareMap.get(DcMotor.class, "motor");



        waitForStart();

        motor.setPower(0.1);
        sleep(2000);
        motor.setPower(-0.1);
        sleep(2000);
        servo.setPosition(0);
        sleep(2000);
        servo.setPosition(1);
        sleep(1000);

        while(opModeIsActive()){
            telemetry.addData("encoder position", motor.getCurrentPosition());
            telemetry.update();


        }

        motor.setPower(0);
        servo.setPosition(0);

    }
}

