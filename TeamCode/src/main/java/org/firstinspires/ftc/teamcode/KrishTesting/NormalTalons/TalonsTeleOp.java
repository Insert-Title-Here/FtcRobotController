package org.firstinspires.ftc.teamcode.KrishTesting.NormalTalons;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.KrishTesting.Command.Talons.RobotT;
import org.firstinspires.ftc.teamcode.KrishTesting.Command.Talons.TalonsIntake;
import org.firstinspires.ftc.teamcode.KrishTesting.Command.Talons.TalonsScoringSystem;
import org.firstinspires.ftc.teamcode.MecanumDriveTrain;
import org.firstinspires.ftc.teamcode.Vector2D;

import java.io.FileNotFoundException;

@TeleOp
public class TalonsTeleOp extends LinearOpMode {

    Robot robot;
    Thread intakeThread;
    Thread scoringThread;


    @Override
    public void runOpMode() throws InterruptedException {

        intakeThread = new Thread(){
            @Override
            public void run(){
                while(opModeIsActive()){
                    intakeUpdate();
                }
            }
        };



        scoringThread = new Thread(){
            @Override
            public void run(){
                while(opModeIsActive()){
                    scoringUpdate();
                }
            }
        };

        initialize();
        waitForStart();

        while(opModeIsActive() && !isStopRequested()){

            robot.drive.setPower(new Vector2D(gamepad1.left_stick_x, gamepad1.left_stick_y), gamepad1.right_stick_x, false);


        }

        onStop();



    }

    public void intakeUpdate(){
        if(gamepad1.a){
            robot.intake.setPower(1);

        }


    }

    public void scoringUpdate(){

    }




    public void initialize(){
        try {
            robot = new Robot(hardwareMap);
        } catch (FileNotFoundException e) {
            e.printStackTrace();
        }
    }

    public void onStop(){
        robot.brake();
    }
}
