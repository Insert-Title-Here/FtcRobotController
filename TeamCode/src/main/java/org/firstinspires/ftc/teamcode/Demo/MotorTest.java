package org.firstinspires.ftc.teamcode.Demo;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class MotorTest extends LinearOpMode {
    DcMotor motor;
    public void runOpMode() throws InterruptedException {
        motor = hardwareMap.get(DcMotor.class, "motor1");
        waitForStart();
        double power = 0;
        boolean AFlag = true;
        while (opModeIsActive()) {
            motor.setPower(power);
            if (gamepad1.a && AFlag){
                power += .05;
                AFlag = false;
            }
            if(!gamepad1.a){
                AFlag = true;
            }


        }
    }
}