package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Common.MecanumDrive;
import org.firstinspires.ftc.teamcode.Common.ScoringSystem;
import org.firstinspires.ftc.teamcode.Common.Vector2D;

import java.util.concurrent.atomic.AtomicBoolean;
@TeleOp
public class SecondTeleOp extends LinearOpMode {
    //TODO: change names if you want to
    MecanumDrive drive;
    ScoringSystem score;
    Thread liftThread;
    AtomicBoolean pause;
    AtomicBoolean discontinue;
    BNO055IMU imu;

    private final double NORMAL_LINEAR_MODIFIER = 0.5;
    private final double NORMAL_ROTATIONAL_MODIFIER = 0.4;
    private final double SPRINT_LINEAR_MODIFIER = 1;
    private final double SPRINT_ROTATIONAL_MODIFIER = 1;
    @Override
    public void runOpMode() throws InterruptedException {
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
        score = new ScoringSystem(hardwareMap, telemetry);
        pause = new AtomicBoolean();
        discontinue = new AtomicBoolean();
        pause.set(true);
        discontinue.set(false);
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        //Open
        //Thread for the slides

        liftThread = new Thread(){
            @Override
            public void run(){
                while(opModeIsActive()){
                    //manually lifts slides up and down
                    if(gamepad1.right_bumper){
                        if(score.getEncoderPosition() > 2390){
                            score.setPower(0);
                        }else{
                            score.setPower(0.6);
                        }
                    }else if(gamepad1.left_bumper) {
                        if(score.getEncoderPosition() < 2){
                            score.setPower(0);
                        }else{
                            score.setPower(-0.7);
                        }
                    }else{
                        if(!discontinue.get()){
                            score.setPower(0.1);
                        }
                    }
                    //moves the slides all the way down
                    if(gamepad1.dpad_down) {
                        score.goToPosition(0, 0.4);
                    }
                    //moves the slides to highest pole height
                    if (gamepad1.right_stick_button) {
                        //high cone
                        score.goToPosition(2330, 0.95);
                        score.setPower(0.1);
                    }
                    //moves slides to medium pole height
                    if(gamepad1.left_stick_button){
                        //medium cone
                        score.goToPosition(1660, 0.95);
                        score.setPower(0.1);
                    }
                    //temporary command that will move the slides to the low pole height
                    if(gamepad1.x){
                        //low cone
                        score.goToPosition(870, 0.8);
                        score.setPower(0.08);
                    }
                    //resets the slidemotor encoder
                    if(gamepad1.options){
                        score = new ScoringSystem(hardwareMap, telemetry);
                    }
                    //closes the claw(manually) and opens the claw(like a toggle)
                    if(gamepad1.right_trigger > 0.1 && pause.get()){
                        if(0.2 < score.getClawPosition() && score.getClawPosition() < 0.34){
                            if(score.getEncoderPosition() > 900){
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
                                    score.setClawPosition(0.32); // TODO: 0.32 is closed, 0 is open (not actually todo)
                                    try {
                                        Thread.sleep(300);
                                    } catch (InterruptedException e) {
                                        e.printStackTrace();
                                    }
                                    score.goToPosition(score.getEncoderPosition()+300,0.6);
                                    discontinue.set(false);
                                }else{
                                    score.setClawPosition(0.32);
                                    try {
                                        Thread.sleep(300);
                                    } catch (InterruptedException e) {
                                        e.printStackTrace();
                                    }
                                    score.goToPosition(120, 0.35);
                                }
                            }else{
                                if(discontinue.get()){
                                    score.setClawPosition(0.32);
                                    try {
                                        Thread.sleep(300);
                                    } catch (InterruptedException e) {
                                        e.printStackTrace();
                                    }
                                    score.goToPosition(score.getEncoderPosition()+330,0.6);
                                    discontinue.set(false);
                                }else{
                                    score.setClawPosition(0.32);
                                }
                            }
                        }
                        //2220
                        pause.set(false);
                    }else if(gamepad1.right_trigger < 0.1){
                        pause.set(true);
                    }
                    // closes claw using color sensor
                    if (score.getClawPosition() == 0.0) {
                        try {
                            score.grabCone(true);
                            pause.set(true);
                        } catch (InterruptedException e) {
                            e.printStackTrace();
                        }
                    }

                }
            }
        };
        score.setClawPosition(0);
        waitForStart();
        liftThread.start();
        int stackHeight = 305;
        boolean stackDoubleDown = true;
        boolean stackDoubleUp = true;
        while(opModeIsActive()){
            //Limits robot movement from controls to only the 4 cardinal directions N,S,W,E
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
            if (gamepad1.left_trigger > 0.1) { // replace this with a button for sprint
                drive.setPower(new Vector2D(gamepadX * SPRINT_LINEAR_MODIFIER, gamepadY * SPRINT_LINEAR_MODIFIER), gamepad1.right_stick_x * SPRINT_ROTATIONAL_MODIFIER, false);
            }else {
                if(score.getEncoderPosition() > 1000){
                    drive.setPower(new Vector2D(gamepadX * 0.4, gamepadY * 0.4), gamepad1.right_stick_x * 0.4, false);
                }else{
                    drive.setPower(new Vector2D(gamepadX * NORMAL_LINEAR_MODIFIER, gamepadY * NORMAL_LINEAR_MODIFIER), gamepad1.right_stick_x * NORMAL_ROTATIONAL_MODIFIER, false);
                }
            }
            //Used for testing purposes-turns a certain number of radians
            if (gamepad1.dpad_left) {
                //turn test
                //drive.turnToInitialPosition();
                drive.turn(Math.PI/4, 0);
            }
            //lowers the height of the slides for the stack of 5 cones
            if(gamepad1.a && stackDoubleDown){
                discontinue.set(true);
                stackDoubleDown = false;
                if((stackHeight - 80) > 0){
                    stackHeight -= 80;
                    score.goToPosition(stackHeight, 0.4);
                }
            }if(gamepad1.a && !stackDoubleDown){
                stackDoubleDown = true;
            }
            //moves the slides to the stack of 5 cones height
            if(gamepad1.y && stackDoubleUp){
                discontinue.set(true);
                stackDoubleUp = false;
                if(stackHeight < 305){
                    score.goToPosition(stackHeight, 0.6);
                    stackHeight +=80;
                }else if(stackHeight == 305){
                    score.goToPosition(305, 0.6);
                }
            }if(gamepad1.y && !stackDoubleUp){
                stackDoubleUp = true;
            }

            //resets the drive motor encoders
            if (gamepad1.share) {
                drive.resetEncoders();
            }

            //TODO: add telemtry for gamepad a and y positions when you press them
            telemetry.addData("flPos", drive.getFLPosition());
            telemetry.addData("frPos", drive.getFRPosition());
            telemetry.addData("blPos", drive.getBLPosition());
            telemetry.addData("brPos", drive.getBRPosition());
            telemetry.addData("Ppower", drive.getProportionPower());
            telemetry.addData("Ipower", drive.getIntegralPower());
            telemetry.addData("Dpower", drive.getDerivativePower());
            telemetry.addData("current angle", imu.getAngularOrientation().firstAngle);
            telemetry.addData("drivePower",(drive.getProportionPower()));
            telemetry.addData("angleError", drive.getAngleError());
            telemetry.addData("realDrivePow", drive.getPower());
            telemetry.addData("liftPos", score.getEncoderPosition());
            telemetry.addData("clawPos", score.getClawPosition());
            telemetry.addData("liftPow", score.getPower());
            telemetry.addData("stackHeight", stackHeight);
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
}