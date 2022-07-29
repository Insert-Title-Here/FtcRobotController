package org.firstinspires.ftc.teamcode.KrishTesting;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.util.ElapsedTime;


import org.firstinspires.ftc.teamcode.Intake;
import org.firstinspires.ftc.teamcode.MecanumDriveTrain;
import org.firstinspires.ftc.teamcode.OpModeWrapper;
import org.firstinspires.ftc.teamcode.Vector2D;

import java.io.FileNotFoundException;

@Disabled
@TeleOp(name="Krish Framework")
public class TestingTeleOpFramework extends LinearOpMode {

    Thread driveThread;
    Thread intakeThread;
    RobotK robot;




    private final double NORMAL_LINEAR_MODIFIER = 0.45;
    private final double NORMAL_ROTATIONAL_MODIFIER = 0.45;
    private final double SPRINT_LINEAR_MODIFIER = 1;
    private final double SPRINT_ROTATIONAL_MODIFIER = 0.75;

    private ElapsedTime timer;
    private boolean timerFlag;
    private boolean changeState;
    protected boolean blueAlliance;



    @Override
    public void runOpMode() throws InterruptedException {

        timerFlag = true;
        changeState = true;
        blueAlliance = true;

        try {
            robot = new RobotK(hardwareMap, telemetry);
        } catch (FileNotFoundException e) {
            e.printStackTrace();
        }



        driveThread = new Thread(){
            @Override
            public void run(){
                while(opModeIsActive()){
                    driveUpdate();
                }
            }
        };



        intakeThread = new Thread(){
            @Override
            public void run(){
                while(opModeIsActive()){
                    //intakeUpdate();
                }
            }
        };

        //robot.color.enableLed(true);
        waitForStart();
        startOpMode();



    }

    public void startOpMode(){
        driveThread.start();
        timer = new ElapsedTime();


        while(opModeIsActive() && !isStopRequested()){


            telemetry.addData("Not in here", timerFlag);

            telemetry.addData("Timer: ", timer);
            telemetry.update();

        }

        robot.drive.brake();

        if(blueAlliance) {
            robot.setColor(0, 0, 255);
        }else{
            robot.setColor(255, 0, 0);
        }
    }



    private void driveUpdate() {
        if (gamepad1.right_bumper) { // replace this with a button for sprint
            robot.drive.setPower(new Vector2D(gamepad1.left_stick_x * SPRINT_LINEAR_MODIFIER, gamepad1.left_stick_y * SPRINT_LINEAR_MODIFIER), gamepad1.right_stick_x * SPRINT_ROTATIONAL_MODIFIER, false);
        } else {
            robot.drive.setPower(new Vector2D(gamepad1.left_stick_x * NORMAL_LINEAR_MODIFIER, gamepad1.left_stick_y * NORMAL_LINEAR_MODIFIER), gamepad1.right_stick_x * NORMAL_ROTATIONAL_MODIFIER, false);
        }
    }

    /*

    private void intakeUpdate(){
        if (gamepad1.a || (robot.color.red() > 130 && robot.color.green() > 150 && robot.color.blue() > 60)) {
            robot.intake.clampAndRelease(true);
        } else {
            robot.intake.clampAndRelease(false);
        }

        if(gamepad1.left_stick_button && changeState){
            robot.intake.shiftConstantState();
            changeState = false;
        }else{
            changeState = true;
        }



        if(robot.intake.constantState == Intake.ConstantState.Still) {
            if (gamepad1.right_trigger > 0.1) {
                robot.intake.setPower(true, gamepad1.right_trigger);
            } else if (gamepad1.left_trigger > 0.1) {
                robot.intake.setPower(false, gamepad1.left_trigger);
            } else {
                robot.intake.setPower(true, 0);
            }
        }else{
            if(robot.intake.constantState == Intake.ConstantState.In){
                robot.intake.setPower(true, 0.75);
            }else if(robot.intake.constantState == Intake.ConstantState.Out){
                robot.intake.setPower(false, 0.75);
            }
        }


    }

     */



}
