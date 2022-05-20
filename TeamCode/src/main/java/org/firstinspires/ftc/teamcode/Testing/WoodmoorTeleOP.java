package org.firstinspires.ftc.teamcode.Testing;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.MecanumCode.Common.CapstoneArm;
import org.firstinspires.ftc.teamcode.MecanumCode.Common.Carousel;
import org.firstinspires.ftc.teamcode.MecanumCode.Common.Constants;
import org.firstinspires.ftc.teamcode.MecanumCode.Common.MagneticArm;
import org.firstinspires.ftc.teamcode.MecanumCode.Common.MecanumDriveTrain;
import org.firstinspires.ftc.teamcode.MecanumCode.Common.Vector2D;

import java.io.FileNotFoundException;


@TeleOp(name="Woodmoor TeleOP")
public class WoodmoorTeleOP extends LinearOpMode {

    MecanumDriveTrain drive;

    Servo grabber, grabber2;



    Thread driveThread;
    Thread armThread;





    /**
     * calibrate all these values kevin
     */

    private final double NORMAL_LINEAR_MODIFIER = 0.45;
    private final double NORMAL_ROTATIONAL_MODIFIER = 0.45;
    private final double SPRINT_LINEAR_MODIFIER = 1;
    private final double SPRINT_ROTATIONAL_MODIFIER = 0.75;
    //private Carousel.CarouselMode carouselSpeed = Carousel.CarouselMode.TELEOP;



    @Override
    public void runOpMode() throws InterruptedException {
        /*
        arm = new MagneticArm(hardwareMap, MagneticArm.OpMode.TeleOp);
        carousel = new Carousel(hardwareMap);
        capArm = new CapstoneArm(hardwareMap);
        timerLB = new ElapsedTime();
        timerY = new ElapsedTime();

         */

        grabber = hardwareMap.get(Servo.class, "Left");
        grabber2 = hardwareMap.get(Servo.class, "Right");

        try {
            drive = new MecanumDriveTrain(hardwareMap);
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


        armThread = new Thread(){
            @Override
            public void run(){
                while(opModeIsActive()){
                    armUpdate();
                }
            }
        };



        waitForStart();
        startOpMode();
    }

    private void startOpMode() {
        driveThread.start();
        armThread.start();
        //grabber.setPosition(0.4);
        //capArmThread.start();
        //arm.setArmPosition(Constants.NEW_MAGARM_RETRACTED);
        while(opModeIsActive());
    }

    private void armUpdate() {
        /**
         * Kevin go implement this
         */


        if(gamepad1.a){
            grabber.setPosition(0.95);
            grabber2.setPosition(0.05);

        }else if(gamepad1.b){
            grabber.setPosition(0.65);
            grabber2.setPosition(0.35);
        }


    }

    private void driveUpdate() {



        /*if(driveSwapped) {
            if (gamepad1.right_bumper) { // replace this with a button for sprint
                drive.setPower(new Vector2D(-gamepad1.left_stick_x * SPRINT_LINEAR_MODIFIER, -gamepad1.left_stick_y * SPRINT_LINEAR_MODIFIER), gamepad1.right_stick_x * SPRINT_ROTATIONAL_MODIFIER, false);
            } else {
                drive.setPower(new Vector2D(-gamepad1.left_stick_x * NORMAL_LINEAR_MODIFIER, -gamepad1.left_stick_y * NORMAL_LINEAR_MODIFIER), gamepad1.right_stick_x * NORMAL_ROTATIONAL_MODIFIER, false);
            }
        }
        else {

        if (gamepad1.right_bumper) { // replace this with a button for sprint
            //drive.setPower(new Vector2D(gamepad1.left_stick_x * SPRINT_LINEAR_MODIFIER, gamepad1.left_stick_y * SPRINT_LINEAR_MODIFIER), gamepad1.right_stick_x * SPRINT_ROTATIONAL_MODIFIER, false);
            drive.setPower(0.3, 0.3, 0.3, 0.3);
            sleep(2000);
        } else {


        }

         */

        drive.setPower(new Vector2D(gamepad1.left_stick_x * NORMAL_LINEAR_MODIFIER, gamepad1.left_stick_y * NORMAL_LINEAR_MODIFIER), gamepad1.right_stick_x * NORMAL_ROTATIONAL_MODIFIER, false);

        //}



        /*if (gamepad1.left_stick_button) {
            drive.driveAuto(0.7, -3665, MecanumDriveTrain.MovementType.ROTATE);
            while (gamepad1.left_stick_button) {

            }
        }

         */
    }

/*
    private void capArmUpdate() throws InterruptedException {

        if(gamepad1.start && !previousStartState) {
            capArm.toggleGrab();
            previousStartState = true;
        }
        if(!gamepad1.start) {
            previousStartState = false;
        }

        if(gamepad1.y) {
            //timerY.reset();
            capArm.goToPosition(0);
            capArm.setGrabberPosition(CapstoneArm.CAP_SERVO_CLOSED);
            Thread.sleep(1000);
            capArm.goToPosition(Constants.NEW_CAPPING_POS);



        }









        if(gamepad1.dpad_up && !previousUpState) {
            if (Math.abs(capArm.getTelemetry()[0] - Constants.CAPPING_POS) < 40) {
                //telemetry.addData("lift", "yeah");
                capArm.goToPosition(Constants.NEW_CAPPING_POS);

            }else if(capArm.getTelemetry()[0] != Constants.CAPPING_POS && capArm.getTelemetry()[0] != Constants.NEW_CAPPING_POS) {
                //telemetry.addData("smalllift", "yeasdfasdf");

                capArm.goToPosition(Constants.CAPPING_POS);
            }
            previousUpState = true;
            //timerUp.reset();



        }else if (gamepad1.dpad_down){
            capArm.goToPosition(-300);
        }


        previousUpState = gamepad1.dpad_up;


        if(gamepad1.dpad_right && capArm.getTelemetry()[0] > Constants.MAX_MANUAL_CAP) {
            //driveSwapped = true;
            capArm.setPower(-0.3);
        }else if(gamepad1.dpad_left && capArm.getTelemetry()[0] < 0) {
            //driveSwapped = false;
            capArm.setPower(0.3);
        }else{
            capArm.setPower(0);
        }


        //capArm.setPower(gamepad2.left_stick_y / 2);
    }
    */
}
