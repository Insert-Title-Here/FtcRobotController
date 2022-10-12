package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Common.MecanumDrive;
import org.firstinspires.ftc.teamcode.Common.ScoringSystem;
import org.firstinspires.ftc.teamcode.Common.Vector2D;

import java.util.concurrent.atomic.AtomicBoolean;

@TeleOp
public class FirstTeleOp extends LinearOpMode {

    //TODO: change names if you want to
    MecanumDrive drive;
    ScoringSystem score;
    Thread liftThread;
    AtomicBoolean cont;


    private final double NORMAL_LINEAR_MODIFIER = 0.5;
    private final double NORMAL_ROTATIONAL_MODIFIER = 0.5;
    private final double SPRINT_LINEAR_MODIFIER = 1;
    private final double SPRINT_ROTATIONAL_MODIFIER = 1;

    @Override
    public void runOpMode() throws InterruptedException {

        drive = new MecanumDrive(hardwareMap, telemetry);
        score = new ScoringSystem(hardwareMap);
        cont = new AtomicBoolean();
        cont.set(false);

        telemetry.addData("Status", "Initialized");
        telemetry.update();


        //score = new ScoringSystem(hardwareMap);

        //Open
        score.setClawPosition(0.9);


        //TODO: Test below Out
        //Comment out code in opmodeisactive while loop if you test this tread out(as well as the thread aboeve)

        liftThread = new Thread(){
            @Override
            public void run(){
                //removed !score.isBusy() from the while statement
                while(opModeIsActive()){
                    if(gamepad1.right_trigger > 0.1){
                        score.setPower(gamepad1.right_trigger/1.6);
                    }else if(gamepad1.left_trigger > 0.1) {
                        if (score.getEncoderPosition() < 40) {
                            //calibrate will set power
                            calibrateLiftBottom(score.getEncoderPosition());
                        } else {
                            if (score.getEncoderPosition() < 100) {
                                score.goToPosition(0, 0.3);
                            } else {
                                score.setPower(-gamepad1.left_trigger / 2);
                            }
                        }
                    }else{
                        score.setPower(0.08);
                    }
                    // reset   gamepad1.dpad_down
                    // low cone, 13 in, 1209  gamenpad1.dpad_left
                    // medium cone, 23 in, 1795 gamepad1.dpad_up
                    // high cone, 33 in, 2390 gamepad1.dpad_right

                    if(gamepad1.dpad_down) {
                        score.goToPosition(0, 0.3);
                    }

                    if (gamepad1.dpad_left) {
                        //low cone
                        score.goToPosition(1209, 1);
                        score.setPower(0.08);
                    }

                    if (gamepad1.dpad_up) {
                        //medium cone
                        score.goToPosition(1795, 1);
                        score.setPower(0.08);
                    }

                    if (gamepad1.dpad_right) {
                        //high cone
                        score.goToPosition(2360, 1);
                        score.setPower(0.08);
                    }
                }
            }
        };





        waitForStart();
        liftThread.start();

        while(opModeIsActive()){


            //TODO: Decide if you want sprint capability
            if (gamepad1.right_bumper) { // replace this with a button for sprint
                drive.setPower(new Vector2D(gamepad1.left_stick_x * SPRINT_LINEAR_MODIFIER, gamepad1.left_stick_y * SPRINT_LINEAR_MODIFIER), gamepad1.right_stick_x * SPRINT_ROTATIONAL_MODIFIER, false);
            }
            else {
                drive.setPower(new Vector2D(gamepad1.left_stick_x * NORMAL_LINEAR_MODIFIER, gamepad1.left_stick_y * NORMAL_LINEAR_MODIFIER), gamepad1.right_stick_x * NORMAL_ROTATIONAL_MODIFIER, false);
            }
            /*
            if(gamepad1.right_trigger > 0.1){
                cont.set(false);
                sleep(30);
                score.setPower(gamepad1.right_trigger/1.6);
            }else if(gamepad1.left_trigger > 0.1){
                cont.set(false);
                sleep(30);
                if(score.getEncoderPosition() < 40){
                    //calibrate will set power
                    calibrateLiftBottom(score.getEncoderPosition());
                }else{
                    if(score.getEncoderPosition() < 100){
                        score.goToPosition(0,0.3);
                    }else{
                        score.setPower(-gamepad1.left_trigger/2);

                    }
                }
            }else{
                cont.set(true);
            }

             */

            if(gamepad1.b){

                //Closed
                score.setClawPosition(0.45);


            //2220

            }else if(gamepad1.x){
                //Open
                score.setClawPosition(0.9);

            }
            if(gamepad1.a){
                drive.resetEncoders();
                //score.resetLiftEncoder();
            }
            if(gamepad1.y){
                cont.set(false);
            }

            // reset   gamepad1.dpad_down
            // low cone, 13 in, 1209  gamenpad1.dpad_left
            // medium cone, 23 in, 1795 gamepad1.dpad_up
            // high cone, 33 in, 2390 gamepad1.dpad_right
            /*
            if(gamepad1.dpad_down) {
                cont.set(false);
                sleep(30);
                //reset
                // 50 not zero b/c 435 motor does not have enough torque to stop the gravitational drop to zero tics
                //score.goToPosition(50, 1);
                //Test this callibration

                while(score.getEncoderPosition() > 700 && score.getEncoderPosition() < 200){
                    int aimedPos = (int)(Math.sqrt(score.getEncoderPosition()));
                    score.goToPosition(aimedPos, 1);
                }


                score.goToPosition(0, 0.3);

            }

            if (gamepad1.dpad_left) {
                cont.set(false);
                sleep(30);
                //low cone
                score.goToPosition(1209, 1);
                cont.set(true);
            }

            if (gamepad1.dpad_up) {
                cont.set(false);
                sleep(30);
                //medium cone
                score.goToPosition(1795, 1);
                cont.set(true);
            }

            if (gamepad1.dpad_right) {
                cont.set(false);
                sleep(30);
                //high cone
                score.goToPosition(2390, 1);
                cont.set(true);
            }

             */

            telemetry.addData("flPos", drive.getFLPosition());
            telemetry.addData("frPos", drive.getFRPosition());
            telemetry.addData("blPos", drive.getBLPosition());
            telemetry.addData("brPos", drive.getBRPosition());
            telemetry.addData("liftPos", score.getEncoderPosition());
            telemetry.update();


        }

        drive.setPower(0, 0, 0, 0);
        score.setPower(0);
        score.setClawPosition(0.9);
    }
    //Test this out
    public void calibrateLiftBottom(int tics){
        if(tics < 70){
            int aimedPow = (int)(Math.sqrt(tics) / 15);
            score.setPower(aimedPow);
        }


    }


}
