package org.firstinspires.ftc.teamcode.League1.Testing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.League1.Common.OpModeWrapper;

import java.io.FileNotFoundException;

@TeleOp
public class ArmTester extends OpModeWrapper {
    Servo lServo, rServo, grabber;
    DcMotor rMotor, lMotor;

    @Override
    protected void onInitialize() throws FileNotFoundException {




        rServo = hardwareMap.get(Servo.class, "right");
        lServo = hardwareMap.get(Servo.class, "left");
        grabber = hardwareMap.get(Servo.class, "grabber");

        rServo.setPosition(0);
        lServo.setPosition(0);
        grabber.setPosition(0.8);

        rMotor = hardwareMap.get(DcMotor.class, "rightM");
        lMotor = hardwareMap.get(DcMotor.class, "leftM");

        rMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        rMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);




    }

    @Override
    protected void onStart() {


        /*
        while(opModeIsActive()){

            if(gamepad1.right_trigger > 0.1){
                rMotor.setPower(gamepad1.right_trigger);
                lMotor.setPower(-gamepad1.right_trigger);

            }else if(gamepad1.left_trigger > 0.1){
                rMotor.setPower(-gamepad1.left_trigger);
                lMotor.setPower(gamepad1.left_trigger);
            }else{
                rMotor.setPower(0);
                lMotor.setPower(0);
            }

        }

         */

        /*for(int i = 0; i <= 1; i += 0.1){
            rServo.setPosition(i);
            lServo.setPosition(i);
            sleep(1000);
        }

         */
        
    }

    @Override
    protected void onStop() {
        /*rServo.setPosition(0);
        lServo.setPosition(0);

         */

    }
}
