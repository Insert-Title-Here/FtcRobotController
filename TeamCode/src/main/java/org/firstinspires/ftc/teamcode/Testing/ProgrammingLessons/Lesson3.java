package org.firstinspires.ftc.teamcode.Testing.ProgrammingLessons;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


import org.firstinspires.ftc.teamcode.Competition.Interleagues.Common.Constants;
/*
Cardinal Drive vs. All Directions
Gamepad done
Buttons done
Triggers done
Sticks
Rumble done
Toggle Logic
Telemetry (update/addData) done

 */
@TeleOp (name = "ProgramingLesson3")
public class Lesson3 extends LinearOpMode {
    MecanumDrive drive;


    @Override
    public void runOpMode() throws InterruptedException {

        drive = new MecanumDrive(hardwareMap, telemetry);

        waitForStart();

        while(opModeIsActive()){
            /*
            Gamepad
             */
           if(gamepad1.right_trigger > 0.05){
               //left trigger or right trigger
           }
           if(gamepad1.right_stick_button){
               //right stick button or left stick button press donw etc.
           }
           if(gamepad1.a){
               // x, y, a, b
           }
           if(gamepad1.dpad_down){
               //dpad_up, dpad_down, dpad_left, dpad_right
           }
           if(gamepad1.isRumbling()){
               gamepad1.rumble(1000);
           }
           /*
            if (gamepad1.share) {
                drive.resetIMU();
            }

            */
          /*
          Toggle Logic
          */
            /*
            if (gamepad1.left_trigger > 0.1 && triggerScoreToggle.get()) {
                if (score.scoreHigh()) {
                    //high cone Max limit is 1370
                    steadyPower = false;
                    score.goToPositionPID(constant.getHeightHigh(), 1);
                    steadyPower = true;
                    score.setPower(constant.getSteadyPow());
                } else if (score.scoreMid()) {
                    //medium cone
                    steadyPower = false;
                    score.goToPositionPID(constant.getHeightMed(), 1);
                    steadyPower = true;
                    score.setPower(constant.getSteadyPow());
                } else if (score.scoreLow()) {
                    //low cone
                    steadyPower = false;
                    score.goToPositionPID(constant.getHeightLow(), 0.8);
                    steadyPower = true;
                    score.setPower(constant.getSteadyPow());
                }
                triggerScoreToggle.set(false);
            } else if (gamepad1.left_trigger < 0.1) {
                triggerScoreToggle.set(true);
            }
             if (gamepad1.right_bumper && uprighterToggle) {
                //cone uprighter
                if (constant.getHeightBottom() == 0) {
                    constant.setHeightBottom(47);
                    score.goToBottom(constant.getHeightBottom(), 0.45);
                    uprighting = false;
                } else if (constant.getHeightBottom() == 47) {
                    constant.setHeightBottom(0);
                    score.goToBottom(constant.getHeightBottom(), 0.45);
                    uprighting = true;
                }
                uprighterToggle = false;

            }else if(!gamepad1.right_bumper){
                uprighterToggle = true;
            }
            */


            /*
            Drive(Cardinal vs. All direction
            */
            /*
             double gamepadX = gamepad1.left_stick_x;
            double gamepadY = gamepad1.left_stick_y;
            if(Math.abs(gamepadX) < Math.abs(gamepadY)){
                gamepadX = 0;
            }else if(Math.abs(gamepadX) > Math.abs(gamepadY)){
                gamepadY = 0;
            }else{
                gamepadX = 0;
                gamepadY = 0;
            }
            //robot movement(using controls/gamepads) with sprint mode
            if(!isTurning) {
                if (gamepad1.left_stick_button) { // replace this with a button for sprint
                    drive.setPower(new Vector2D(gamepadX * SPRINT_LINEAR_MODIFIER, gamepadY * SPRINT_LINEAR_MODIFIER), gamepad1.right_stick_x * SPRINT_ROTATIONAL_MODIFIER, false);
                } else {
                    if (score.getEncoderPosition() > 480) {
                        drive.setPower(new Vector2D(gamepadX * 0.55, gamepadY * 0.55), gamepad1.right_stick_x * 0.2, false);
                    } else {
                        drive.setPower(new Vector2D(gamepadX * NORMAL_LINEAR_MODIFIER, gamepadY * NORMAL_LINEAR_MODIFIER), gamepad1.right_stick_x * NORMAL_ROTATIONAL_MODIFIER, false);
                    }
                }
            }
              */
           /*
           Telemetry
           */
           telemetry.addData("Left Motor Power", 0.5);
           telemetry.addData("Right Motor Power", 0.5);
           telemetry.addData("Left motor position", 1200);
           telemetry.update();


        }
        //Stop motors here(set power to zero)
    }
}

