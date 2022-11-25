package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Common.Constants;
import org.firstinspires.ftc.teamcode.Common.MecanumDrive;
import org.firstinspires.ftc.teamcode.Common.ScoringSystem;
import org.firstinspires.ftc.teamcode.Common.Vector2D;

import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicInteger;

@TeleOp
public class SecondTeleOp extends LinearOpMode {
    //TODO: change names if you want to
    MecanumDrive drive;
    ScoringSystem score;
    Thread liftThread;
    AtomicBoolean pause;
    AtomicBoolean discontinue;
    AtomicBoolean reToggle;
    AtomicInteger stackHeight;
    AtomicBoolean stackDoubleDown;
    AtomicBoolean stackDoubleUp;
    AtomicBoolean zero;
    Constants constant;
    BNO055IMU imu;
    String s;

    private final double NORMAL_LINEAR_MODIFIER = 0.65;
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
        zero = new AtomicBoolean();
        pause = new AtomicBoolean();
        constant = new Constants();
        reToggle = new AtomicBoolean();
        discontinue = new AtomicBoolean();
        stackHeight = new AtomicInteger();
        stackDoubleUp = new AtomicBoolean();
        stackDoubleDown = new AtomicBoolean();
        stackDoubleDown.set(true);
        stackDoubleUp.set(true);
        stackHeight.set(constant.getStackHeight());
        reToggle.set(false);
        pause.set(true);
        zero.set(false);
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
                        //if bumper get to max height sets power stop moving up
                        if(score.getEncoderPosition() > constant.getHeightLimit()){
                            score.setPower(constant.getSteadyPow());
                        }else{
                            score.setPower(0.9);
                        }
                    }else if(gamepad1.left_bumper) {
                        if(score.getEncoderPosition() < 7){
                            score.setPower(0);
                        }else{
                            score.setPower(-0.4);
                        }
                    }else{
                        if(score.getEncoderPosition() > 100){
                            if(!discontinue.get()){
                                score.setPower(constant.getSteadyPow());
                            }
                        }
                    }
                    //moves the slides all the way down
                    if(gamepad1.dpad_down) {
                        score.goToPosition(constant.getHeightBottom(),1);
                        zero.set(true);
                    }
                    //moves the slides to highest pole height
                    if (gamepad1.right_stick_button) {
                        //high cone Max limit is 1370
                        score.goToPosition(constant.getHeightHigh(), 0.95);
                        score.setPower(constant.getSteadyPow());
                    }
                    //moves slides to medium pole height
                    if(gamepad1.left_stick_button){
                        //medium cone
                        score.goToPosition(constant.getHeightMed(), 0.95);
                        score.setPower(constant.getSteadyPow());
                    }
                    //temporary command that will move the slides to the low pole height
                    if(gamepad1.x){
                        //low cone
                        score.goToPosition(constant.getHeightLow(), 0.8);
                        score.setPower(constant.getSteadyPow());
                    }
                    //resets the slidemotor encoder
                    if(gamepad1.options){
                        score = new ScoringSystem(hardwareMap, telemetry);
                    }
                    //lowers the height of the slides for the stack of 5 cones
                    if(gamepad1.a && stackDoubleDown.get()){
                        discontinue.set(true);
                        stackDoubleDown.set(false);
                        if((stackHeight.get() - constant.getStackIntervalHeight()) > 0){
                            stackHeight.set(stackHeight.get()-constant.getStackIntervalHeight());
                            score.goToPosition(stackHeight.get(), 0.8);
                        }
                    }else if(!gamepad1.a){
                        stackDoubleDown.set(true);
                    }
                    //moves the slides to the stack of 5 cones height
                    if(gamepad1.y && stackDoubleUp.get()){
                        discontinue.set(true);
                        stackDoubleUp.set(false);
                        if(stackHeight.get() < constant.getStackHeight()){
                            score.goToPosition(stackHeight.get(), 0.8);
                            stackHeight.set(stackHeight.get() + constant.getStackIntervalHeight());
                        }else if(stackHeight.get() == constant.getStackHeight()){
                            score.goToPosition(constant.getStackHeight(), 0.8);
                        }
                    }else if(!gamepad1.y){
                        stackDoubleUp.set(true);
                    }
                    //closes the claw(manually) and opens the claw(like a toggle)
                    if(gamepad1.right_trigger > 0.1 && pause.get()){
                        if(constant.getClawLowThreshold() < score.getClawPosition() && score.getClawPosition() < constant.getClawHighThreshold()){
                            //if liftpos is low height ish or higher
                            if(score.getEncoderPosition() > constant.getHeightLow() - 80){
                                if(reToggle.get()){
                                    score.setClawPosition(constant.getClawOpenPos());
                                    try {
                                        Thread.sleep(800);
                                    } catch (InterruptedException e) {
                                        e.printStackTrace();
                                    }
                                    score.goToPosition(constant.getHeightBottom(),1);
                                    zero.set(true);
                                    reToggle.set(false);
                                }else{
                                    score.goToPosition(score.getEncoderPosition() - 113, 1);
                                    reToggle.set(true);
                                }
                            }else{
                                score.setClawPosition(0);
                                try {
                                    Thread.sleep(700);
                                } catch (InterruptedException e) {
                                    e.printStackTrace();
                                }
                                score.goToPosition(constant.getHeightBottom(),0.8);
                                zero.set(true);
                            }

                        }else{
                            //if lifts system is below 95
                            if(score.getEncoderPosition() < 95){
                                //for stack only
                                if(discontinue.get()){
                                    score.setClawPosition(constant.getClawClosePos());
                                    try {
                                        Thread.sleep(600);
                                    } catch (InterruptedException e) {
                                        e.printStackTrace();
                                    }
                                    score.goToPosition(score.getEncoderPosition()+113,1);
                                    score.setPower(constant.getSteadyPow());
                                    discontinue.set(false);
                                }else{
                                    score.setClawPosition(constant.getClawClosePos());
                                    try {
                                        Thread.sleep(600);
                                    } catch (InterruptedException e) {
                                        e.printStackTrace();
                                    }
                                    score.goToPosition(score.getEncoderPosition()+43, 1);
                                    score.setPower(constant.getSteadyPow());
                                    pause.set(false);

                                }
                            }else{
                                if(discontinue.get()){
                                    score.setClawPosition(constant.getClawClosePos());
                                    try {
                                        Thread.sleep(600);
                                    } catch (InterruptedException e) {
                                        e.printStackTrace();
                                    }
                                    score.goToPosition(score.getEncoderPosition()+125,1);
                                    score.setPower(constant.getSteadyPow());
                                    discontinue.set(false);
                                }else{
                                    score.setClawPosition(constant.getClawClosePos());
                                }
                            }
                        }
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
                if(score.getEncoderPosition() > 500){
                    drive.setPower(new Vector2D(gamepadX * 0.4, gamepadY * 0.4), gamepad1.right_stick_x * 0.4, false);
                }else{
                    drive.setPower(new Vector2D(gamepadX * NORMAL_LINEAR_MODIFIER, gamepadY * NORMAL_LINEAR_MODIFIER), gamepad1.right_stick_x * NORMAL_ROTATIONAL_MODIFIER, false);
                }
            }
            //Used for testing purposes-turns a certain number of radians
            if (gamepad1.dpad_left) {
                //turn test
                //drive.turnToInitialPosition();
                drive.turn(-Math.PI/4, 0);
            }
            if(gamepad1.dpad_up){
                drive.goToPosition(0, 0, 0, 0, 2000, "test");
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
            telemetry.addData("current angle", imu.getAngularOrientation().firstAngle);
            telemetry.addData("realDrivePow", drive.getPower());
            telemetry.addData("liftPos", score.getEncoderPosition());
            telemetry.addData("clawPos", score.getClawPosition());
            telemetry.addData("liftPow", score.getPower());
            telemetry.addData("stack", stackHeight.get());
            //// telemetry.addData("blue", color.currentBlueColor());
            //telemetry.addData("red", color.currentRedColor());
            //  telemetry.update();
            //telemetry.update();
            telemetry.update();



        }
        drive.writeLoggerToFile();
        score.writeLoggerToFile();
        drive.setPower(0, 0, 0, 0);
        score.setPower(0);
        score.setClawPosition(1);
    }
}