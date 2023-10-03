package org.firstinspires.ftc.teamcode.Testing.ProgrammingLessons;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;

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




    @Override
    public void runOpMode() throws InterruptedException {




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
          Toggle Logic
          */

            
            /*
            Drive(Cardinal vs. All direction
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

