package org.firstinspires.ftc.teamcode.League1.Testing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.checkerframework.checker.units.qual.C;
import org.firstinspires.ftc.teamcode.League1.Common.Constants;
import org.firstinspires.ftc.teamcode.League1.Common.OpModeWrapper;
import org.firstinspires.ftc.teamcode.League1.Subsystems.ScoringSystem;

import java.io.FileNotFoundException;

@TeleOp
public class ArmTester extends OpModeWrapper {
    //Servo lServo, rServo, grabber;
    //DcMotor rMotor, lMotor;

    ScoringSystem score;
    Constants constants;

    @Override
    protected void onInitialize() throws FileNotFoundException {
        constants = new Constants();
        score = new ScoringSystem(hardwareMap, constants, false);

        score.setLinkagePosition(0.03);
        score.setGrabberPosition(0.75);


        /*
        rServo = hardwareMap.get(Servo.class, "RightLinkage");
        lServo = hardwareMap.get(Servo.class, "LeftLinkage");
        grabber = hardwareMap.get(Servo.class, "Grabber");

        rServo.setPosition(0.03);
        lServo.setPosition(0.03);
        grabber.setPosition(0.8);

         */

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
                score.setLinkagePosition(0.7);
            }

            if(gamepad1.dpad_right) {
                score.setLinkagePosition(1);
            }

            if(gamepad1.dpad_down) {
                score.setLinkagePosition(0.1);

                sleep(1000);

                score.setLinkagePosition(0.03);



            }

            if(gamepad1.dpad_left) {
                score.setGrabberPosition(0.5);
            }

            telemetry.addData("Linkage Pos:", score.getRightLinkage());
            telemetry.addData("Grabber Pos:", score.getGrabberPosition());
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

        score.setLinkagePosition(0.1);
        sleep(200);
        score.setLinkagePosition(0.03);
        score.setGrabberPosition(0.75);


    }
}
