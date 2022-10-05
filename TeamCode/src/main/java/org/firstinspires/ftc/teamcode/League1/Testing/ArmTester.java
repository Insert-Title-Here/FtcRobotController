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




        rServo = hardwareMap.get(Servo.class, "RightLinkage");
        lServo = hardwareMap.get(Servo.class, "LeftLinkage");
        grabber = hardwareMap.get(Servo.class, "Grabber");

        rServo.setPosition(0.03);
        lServo.setPosition(0.03);
        grabber.setPosition(0.8);

        //rMotor = hardwareMap.get(DcMotor.class, "rightM");
        //lMotor = hardwareMap.get(DcMotor.class, "leftM");

        //rMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //lMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //rMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //lMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);




    }

    @Override
    protected void onStart() {



        while(opModeIsActive()) {
            /*

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

             */

            if(gamepad1.dpad_up) {
                rServo.setPosition(0.7);
                lServo.setPosition(0.7);
            }

            if(gamepad1.dpad_right) {
                rServo.setPosition(1);
                lServo.setPosition(1);
            }

            if(gamepad1.dpad_down) {
                rServo.setPosition(0.1);
                lServo.setPosition(0.1);

                sleep(1000);

                rServo.setPosition(0.03);
                lServo.setPosition(0.03);



            }

            if(gamepad1.dpad_left) {
                grabber.setPosition(0.5);
            }

            telemetry.addData("Linkage Pos:", rServo.getPosition());
            telemetry.addData("Grabber Pos:", grabber.getPosition());
            telemetry.update();

        }



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
