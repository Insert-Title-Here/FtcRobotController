package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Common.MecanumDrive;
import org.firstinspires.ftc.teamcode.Common.ScoringSystem;
import org.firstinspires.ftc.teamcode.Common.Vector2D;

import java.util.Formattable;
import java.util.concurrent.atomic.AtomicBoolean;

@TeleOp
public class FirstTeleOp extends LinearOpMode {

    //TODO: change names if you want to
    MecanumDrive drive;
    ScoringSystem score;
    Thread liftThread;
    AtomicBoolean pause;

    private final double NORMAL_LINEAR_MODIFIER = 0.7;
    private final double NORMAL_ROTATIONAL_MODIFIER = 0.45;
    private final double SPRINT_LINEAR_MODIFIER = 1;
    private final double SPRINT_ROTATIONAL_MODIFIER = 1;

    @Override
    public void runOpMode() throws InterruptedException {

        drive = new MecanumDrive(hardwareMap, telemetry);
        score = new ScoringSystem(hardwareMap);
        pause = new AtomicBoolean();
        pause.set(true);
        telemetry.addData("Status", "Initialized");
        telemetry.update();


        //score = new ScoringSystem(hardwareMap);

        //Open
        score.setClawPosition(0);

        //TODO: Test below Out
        //Comment out code in opmodeisactive while loop if you test this tread out(as well as the thread aboeve)

        liftThread = new Thread(){
            @Override
            public void run(){
                //removed !score.isBusy() from the while statement
                while(opModeIsActive()){
                    if(gamepad1.right_bumper){
                        score.setPower(1);
                    }else if(gamepad1.left_bumper) {
                        if(score.getEncoderPosition() < 2){
                            score.setPower(0);
                        }else{
                            score.setPower(-0.5);
                        }
                    }else{
                        score.setPower(0.08);
                    }
                    // reset   gamepad1.dpad_down
                    // low cone, 13 in, 1209  gamenpad1.dpad_left
                    // medium cone, 23 in, 1795 gamepad1.dpad_up
                    // high cone, 33 in, 2390 gamepad1.dpad_right

                    if(gamepad1.dpad_down) {
                        score.goToPosition(0, 0.65);
                    }

                    if (gamepad1.dpad_left) {
                        //low cone
                        score.goToPosition(970, 1);
                        score.setPower(0.08);
                    }

                    if (gamepad1.right_stick_button) {
                        //high cone
                        score.goToPosition(2330, 1);
                        score.setPower(0.08);
                    }
                    if(gamepad1.left_stick_button){
                        //medium cone
                        score.goToPosition(1660, 1);
                        score.setPower(0.08);
                    }
                    if(gamepad1.right_trigger > 0.1 && pause.get()){
                        if(0.22 < score.getClawPosition() && score.getClawPosition() < 0.26){
                            score.setClawPosition(0);
                            FirstTeleOp.this.sleep(200);
                            score.goToPosition(0, 0.5);
                        }else{
                            if(score.getEncoderPosition() < 200){
                                score.setClawPosition(0.24);
                                FirstTeleOp.this.sleep(400);
                                score.goToPosition(50, 0.35);
                            }else{
                                score.setClawPosition(0.24);
                            }

                        }
                        //2220
                        pause.set(false);
                    }else if(gamepad1.right_trigger < 0.1){
                        pause.set(true);
                    }

                }
            }
        };

        waitForStart();
        liftThread.start();

        while(opModeIsActive()){
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
            //TODO: Decide if you want sprint capability
            if (gamepad1.left_trigger > 0.1) { // replace this with a button for sprint
                drive.setPower(new Vector2D(gamepadX * SPRINT_LINEAR_MODIFIER, gamepadY * SPRINT_LINEAR_MODIFIER), gamepad1.right_stick_x * SPRINT_ROTATIONAL_MODIFIER, false);
            }else {
                if(score.getEncoderPosition() > 900){
                    drive.setPower(new Vector2D(gamepadX * 0.5, gamepadY * 0.5), gamepad1.right_stick_x * 0.5, false);
                }else{
                    drive.setPower(new Vector2D(gamepadX * NORMAL_LINEAR_MODIFIER, gamepadY * NORMAL_LINEAR_MODIFIER), gamepad1.right_stick_x * NORMAL_ROTATIONAL_MODIFIER, false);
                }
            }


            if(gamepad1.a){
               score.goToPosition(score.getEncoderPosition() - 5, 0.7);
            }
            if(gamepad1.y){
                score.goToPosition(20, 0.7);
            }
            if(gamepad1.options){
                score.resetLiftEncoder();
            }
            if (gamepad1.share) {
                drive.resetEncoders();
            }

            //TODO: add telemtry for gamepad a and y positions when you press them
            telemetry.addData("flPos", drive.getFLPosition());
            telemetry.addData("frPos", drive.getFRPosition());
            telemetry.addData("blPos", drive.getBLPosition());
            telemetry.addData("brPos", drive.getBRPosition());
            telemetry.addData("flPow", drive.getPower());
            telemetry.addData("liftPos", score.getEncoderPosition());
            telemetry.addData("clawPos", score.getClawPosition());
            telemetry.addData("liftPow", score.getPower());
            telemetry.update();


        }

        drive.setPower(0, 0, 0, 0);
        score.setPower(0);
        score.setClawPosition(1);
    }
    //Test this out
    public void calibrateLiftBottom(int tics) {
        if (tics < 70) {
            int aimedPow = (int) (Math.sqrt(tics) / 15);
            score.setPower(aimedPow);
        }

    }

}
