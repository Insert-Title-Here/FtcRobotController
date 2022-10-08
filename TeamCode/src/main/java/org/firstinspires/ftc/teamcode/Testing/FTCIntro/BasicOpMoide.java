package org.firstinspires.ftc.teamcode.Testing.FTCIntro;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

public class BasicOpMoide extends LinearOpMode {


    DcMotor motor;
    Servo servo;

    @Override
    public void runOpMode() throws InterruptedException {
        onInit();
        waitForStart();
        onStart();
    }

    private void onInit() {
        motor = hardwareMap.dcMotor.get("motor");
        motor = hardwareMap.get(DcMotor.class, "motor");
        servo = hardwareMap.servo.get("servo");
    }

    private void onStart() {
        motor.setPower(1.0); //0 and 1, % of the motors max voltage draw
        servo.setPosition(0.9); //<- PWM
        servo.getPosition();
        if(gamepad1.dpad_down){
            //do something
        }else if(gamepad1.left_trigger > 0.3){
            //do something
        }
    }
}
