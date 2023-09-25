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
Gamepad
Buttons
Triggers
Sticks
Rumble
Toggle Logic
Telemetry (update/addData)

 */
@TeleOp (name = "ProgramingLesson3")
public class Lesson3 extends LinearOpMode {

    CRServo left,right;


    @Override
    public void runOpMode() throws InterruptedException {

        left = hardwareMap.get(CRServo.class, "leftServo");
        right = hardwareMap.get(CRServo.class, "rightServo");


        waitForStart();
        

        while(opModeIsActive()){
            if(gamepad1.left_trigger > 0.06){
                left.setPower(-gamepad1.left_trigger);
            }else{
                left.setPower(0);
            }
            if(gamepad1.right_trigger > 0.06){
                right.setPower(gamepad1.right_trigger);
            }else{
                right.setPower(0);
            }
        }
        left.setPower(0);
        right.setPower(0);

    }
}

