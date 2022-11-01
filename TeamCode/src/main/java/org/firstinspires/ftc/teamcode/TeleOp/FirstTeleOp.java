package org.firstinspires.ftc.teamcode.TeleOp;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.vuforia.COORDINATE_SYSTEM_TYPE;

import org.firstinspires.ftc.teamcode.Common.ColorSensor;
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
    AtomicBoolean pause;
    BNO055IMU imu;
    AtomicBoolean discontinue;
    //ColorSensor color;
    private final double NORMAL_LINEAR_MODIFIER = 0.7;
    private final double NORMAL_ROTATIONAL_MODIFIER = 0.7;
    private final double SPRINT_LINEAR_MODIFIER = 1;
    private final double SPRINT_ROTATIONAL_MODIFIER = 1;
    @Override
    public void runOpMode() throws InterruptedException {
        double initialAngle;
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // seehe calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        drive = new MecanumDrive(hardwareMap, telemetry);
        score = new ScoringSystem(hardwareMap);
        pause = new AtomicBoolean();
        discontinue = new AtomicBoolean();
        //color = new ColorSensor(hardwareMap, telemetry);
        pause.set(true);
        discontinue.set(false);
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
                        score.setPower(0.6);
                        if(score.getEncoderPosition() > 2390){
                            score.setPower(0.1);
                        }else{
                            score.setPower(1);
                        }
                    }else if(gamepad1.left_bumper) {
                        if(score.getEncoderPosition() < 2){
                            score.setPower(0);
                        }else{
                            score.setPower(-0.5);
                        }
                    }else{
                        score.setPower(0.1);
                    }
                    // reset   gamepad1.dpad_down
                    // low cone, 13 in, 1209  gamenpad1.dpad_left
                    // medium cone, 23 in, 1795 gamepad1.dpad_up
                    // high cone, 33 in, 2390 gamepad1.dpad_right
                    if(gamepad1.dpad_down) {
                        score.goToPosition(0, 0.4);
                    }
                    if (gamepad1.right_stick_button) {
                        //high cone
                        score.goToPosition(2330, 0.95);
                        score.setPower(0.1);
                    }
                    if(gamepad1.left_stick_button){
                        //medium cone
                        score.goToPosition(1660, 0.95);
                        score.setPower(0.1);
                    }
                    if(gamepad1.right_trigger > 0.1 && pause.get()){
                        if(0.22 < score.getClawPosition() && score.getClawPosition() < 0.26){
                            if(score.getEncoderPosition() > 1500){
                                score.goToPosition(score.getEncoderPosition() - 300, 0.5);
                            }
                            score.setClawPosition(0);

                            try {
                                Thread.sleep(800);
                            } catch (InterruptedException e) {
                                e.printStackTrace();
                            }
                            score.goToPosition(0, 0.4);
                        }else{
                            if(score.getEncoderPosition() < 200){
                                if(discontinue.get()){
                                    score.setClawPosition(0.25);
                                    try {
                                        Thread.sleep(300);
                                    } catch (InterruptedException e) {
                                        e.printStackTrace();
                                    }
                                    score.goToPosition(score.getEncoderPosition()+300,0.6);
                                    discontinue.set(false);
                                }else{
                                    score.setClawPosition(0.25);
                                    try {
                                        Thread.sleep(300);
                                    } catch (InterruptedException e) {
                                        e.printStackTrace();
                                    }
                                    score.goToPosition(120, 0.35);
                                }
                            }else{
                                if(discontinue.get()){
                                    score.setClawPosition(0.25);
                                    try {
                                        Thread.sleep(300);
                                    } catch (InterruptedException e) {
                                        e.printStackTrace();
                                    }
                                    score.goToPosition(score.getEncoderPosition()+300,0.6);
                                    discontinue.set(false);
                                }else{
                                    score.setClawPosition(0.25);
                                }
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
        int stackHeight = 300;
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
                if(score.getEncoderPosition() > 1000){
                    drive.setPower(new Vector2D(gamepadX * 0.4, gamepadY * 0.4), gamepad1.right_stick_x * 0.4, false);
                }else{
                    drive.setPower(new Vector2D(gamepadX * NORMAL_LINEAR_MODIFIER, gamepadY * NORMAL_LINEAR_MODIFIER), gamepad1.right_stick_x * NORMAL_ROTATIONAL_MODIFIER, false);
                }
            }
            if (gamepad1.dpad_left) {
                //turn test
                //drive.turnToInitialPosition();
                drive.turn(1.57, 0.5);
            }
            if(gamepad1.a){
                discontinue.set(true);
                if((score.getEncoderPosition() - 30) > 0){
                    score.goToPosition(stackHeight - 30, 0.7);
                    stackHeight = stackHeight - 30;
                }
            }
            if(gamepad1.y){
                discontinue.set(true);
                if(stackHeight != 320){
                    stackHeight += 30;
                }else{
                    score.goToPosition(320, 0.7);
                }
            }
            if(gamepad1.x){
                //low position
                score.goToPosition(1120, 1);
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
            telemetry.addData("current angle", imu.getAngularOrientation().firstAngle);
           //// telemetry.addData("blue", color.currentBlueColor());
            //telemetry.addData("red", color.currentRedColor());
          //  telemetry.update();
            //telemetry.update();
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