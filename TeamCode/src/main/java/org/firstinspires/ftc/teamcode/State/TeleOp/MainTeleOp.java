package org.firstinspires.ftc.teamcode.State.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.State.Auto.ContourMultiScoreLeft;
import org.firstinspires.ftc.teamcode.State.Common.Constants;
import org.firstinspires.ftc.teamcode.State.Common.MecanumDrive;
import org.firstinspires.ftc.teamcode.State.Common.ScoringSystem;
import org.firstinspires.ftc.teamcode.State.Common.Vector2D;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.concurrent.atomic.AtomicBoolean;

@TeleOp @Config
// This teleop does not contain the colorsensor code
public class MainTeleOp extends LinearOpMode {
    //TODO: change names if you want to
    MecanumDrive drive;
    ScoringSystem score;
    Thread liftThread;
    Thread imuThread;
    Constants constant;
    BNO055IMU imu;
    OpenCvWebcam webcam;
    ContourMultiScoreLeft detect1;

    AtomicBoolean clawOpenCloseToggle;
    AtomicBoolean clawStackFlag;
    AtomicBoolean clawMoveDownToggle;
    AtomicBoolean stackFlag;
    AtomicBoolean triggerScoreToggle;

    private boolean uprighterToggle = true;
    private volatile boolean steadyPower = true;
    volatile boolean isTurning = false;
    volatile boolean isTurnLifting = false;
    volatile boolean scoring = false;

    //private double properCX = 187; //67
    //private double properCXLow = 163; //160


    private final double NORMAL_LINEAR_MODIFIER = 0.7;
    private final double NORMAL_ROTATIONAL_MODIFIER = 0.3;
    private final double SPRINT_LINEAR_MODIFIER = 1;
    private final double SPRINT_ROTATIONAL_MODIFIER = 0.75;

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

        //detect1 = new ContourMultiScoreLeft(telemetry);
        drive = new MecanumDrive(hardwareMap, telemetry, true);
        score = new ScoringSystem(hardwareMap, telemetry);
        clawOpenCloseToggle = new AtomicBoolean();
        constant = new Constants();
        clawMoveDownToggle = new AtomicBoolean();
        clawStackFlag = new AtomicBoolean();
        stackFlag = new AtomicBoolean();
        triggerScoreToggle = new AtomicBoolean();

        triggerScoreToggle.set(true);
        stackFlag.set(true);
        clawMoveDownToggle.set(false);
        clawOpenCloseToggle.set(true);
        clawStackFlag.set(false);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        webcam.setPipeline(detect1);
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(320, 176, OpenCvCameraRotation.UPRIGHT);
            }
            @Override
            public void onError(int errorCode) {
                telemetry.addData("Camera Init Error", errorCode);
                telemetry.update();
            }
        });



        //ftc dashboard

        FtcDashboard.getInstance().startCameraStream(webcam, 0);

        //Thread for the slides
        liftThread = new Thread(){
            @Override
            public void run() {
                if (!isTurnLifting) {
                    while (opModeIsActive()) {
                        //manually lifts slides up and down
                        if (gamepad1.dpad_right) {
                            //if bumper get to max height sets power stop moving up
                            if (score.getEncoderPosition() > constant.getHeightLimit()) {
                                score.setPower(constant.getSteadyPow());
                            } else {
                                score.setPower(0.7);
                            }
                        } else if (gamepad1.dpad_left) {
                            score.setPower(-0.24);
                        } else {
                            if(steadyPower) {
                                if (score.getEncoderPosition() > 20) {
                                    score.setPower(constant.getSteadyPow());
                                } else {
                                    score.setPower(0);
                                }
                            }
                        }

                        //TODO: NEW CODING IDEA SCORING!!!
                        // Ideas is to  in tele, use camera to score when close enough to pole

                        // normal scoring
                        if (gamepad1.left_trigger > 0.1 && triggerScoreToggle.get()) {
                            if (score.scoreHigh()) {
                                //high cone Max limit is 1370
                                steadyPower = false;
                                score.goToPositionPID(constant.getHeightHigh(), 1);
                                steadyPower = true;
                                score.setPower(constant.getSteadyPow());
                            } else if (score.scoreMid()) {
                                //medium cone
                                steadyPower = false;
                                score.goToPositionPID(constant.getHeightMed(), 1);
                                steadyPower = true;
                                score.setPower(constant.getSteadyPow());
                            } else if (score.scoreLow()) {
                                //low cone
                                steadyPower = false;
                                score.goToPositionPID(constant.getHeightLow(), 0.8);
                                steadyPower = true;
                                score.setPower(constant.getSteadyPow());
                            }
                            triggerScoreToggle.set(false);
                        } else if (gamepad1.left_trigger < 0.1) {
                            triggerScoreToggle.set(true);
                        }


                        //resets the slidemotor encoder
                        if (gamepad1.options) {
                            score = new ScoringSystem(hardwareMap, telemetry);
                            drive.resetEncoders();
                        }

                        //stack code
                        if (gamepad1.dpad_up && stackFlag.get()) {
                            clawStackFlag.set(true);
                            stackFlag.set(false);
                            score.stackUp();
                        } else if (gamepad1.dpad_down && stackFlag.get()) {
                            clawStackFlag.set(true);
                            stackFlag.set(false);
                            score.stackDown();
                        } else if (!gamepad1.dpad_up && !gamepad1.dpad_down) {
                            stackFlag.set(true);
                        }


                        //closes the claw(manually) and opens the claw(like a toggle)
                        if (gamepad1.right_trigger > 0.1 && clawOpenCloseToggle.get()) {
                            //Checks if claw position is between 2 values to check if claw should open or close
                            if (constant.getClawLowThreshold() < score.getClawPosition() && score.getClawPosition() < constant.getClawHighThreshold()) {
                                //if liftpos is low height ish or higher
                                if (score.getEncoderPosition() > constant.getHeightLow() - 80) {
                                    if (clawMoveDownToggle.get()) {
                                        score.setClawPosition(constant.getClawOpenPos());
                                        try {
                                            Thread.sleep(800);
                                        } catch (InterruptedException e) {
                                            e.printStackTrace();
                                        }
                                        score.goToPosition(constant.getHeightBottom(), 0.7);
                                        clawMoveDownToggle.set(false);
                                    } else {
                                        score.goToPosition(score.getEncoderPosition() - 100, 0.4);
                                        clawMoveDownToggle.set(true);
                                    }
                                } else {
                                    score.setClawPosition(0);
                                    try {
                                        Thread.sleep(700);
                                    } catch (InterruptedException e) {
                                        e.printStackTrace();
                                    }
                                    score.goToPosition(constant.getHeightBottom(), 0.4);
                                }

                            } else {
                                //if lifts system is below 95
                                if (score.getEncoderPosition() < 95) {
                                    //claw code for stack only
                                    if (clawStackFlag.get()) {
                                        score.setClawPosition(constant.getClawClosePos());
                                        try {
                                            Thread.sleep(600);
                                        } catch (InterruptedException e) {
                                            e.printStackTrace();
                                        }
                                        score.goToPosition(score.getEncoderPosition() + 100, 0.8);
                                        score.setPower(constant.getSteadyPow());
                                        clawStackFlag.set(false);

                                    } else {
                                        score.setClawPosition(constant.getClawClosePos());
                                        try {
                                            Thread.sleep(600);
                                        } catch (InterruptedException e) {
                                            e.printStackTrace();
                                        }
                                        score.goToPosition(constant.getHeightLow(), 0.8);
                                        score.setPower(constant.getSteadyPow());
                                        clawOpenCloseToggle.set(false);
                                    /*
                                    if(uprighting){
                                        score.goToPosition(score.getEncoderPosition()+300, 1);
                                        score.setPower(constant.getSteadyPow());
                                        score.setUprighterPosition(0);
                                        clawOpenCloseToggle.set(false);
                                    }else if(!uprighting){
                                        score.goToPosition(score.getEncoderPosition()+100, 1);
                                        score.setPower(constant.getSteadyPow());
                                        clawOpenCloseToggle.set(false);
                                    }

                                     */


                                    }
                                } else {
                                    //claw code for stack only
                                    if (clawStackFlag.get()) {
                                        score.setClawPosition(constant.getClawClosePos());
                                        try {
                                            Thread.sleep(600);
                                        } catch (InterruptedException e) {
                                            e.printStackTrace();
                                        }
                                        score.goToPosition(score.getEncoderPosition() + 150, 0.8);
                                        score.setPower(constant.getSteadyPow());
                                        clawStackFlag.set(false);
                                    } else {
                                        score.setClawPosition(constant.getClawClosePos());
                                    }
                                }
                            }
                            clawOpenCloseToggle.set(false);
                        } else if (gamepad1.right_trigger < 0.1) {
                            clawOpenCloseToggle.set(true);
                        }
                        // closes claw using color sensor
                        if (score.getClawPosition() == 0.0) {
                            try {
                                if (clawStackFlag.get()) {
                                    score.grabCone(true);
                                } else {
                                    score.grabCone(false);
                                }
                                clawOpenCloseToggle.set(true);
                            } catch (InterruptedException e) {
                                e.printStackTrace();
                            }
                        }
                    /*
                    if (detect1.getcX() > properCXLow - 5 && detect1.getcX() < properCXLow + 5 && score.getEncoderPosition() > 12) {
                        if (detect1.getBoundArea() >= 8000.0 && detect1.getBoundArea() <= 9500 && detect1.getDistance() <= 4.3) {
                            drive.goToPosition(0, 0, 0, 0);
                            score.setClawPosition(constant.getClawOpenPos());
                            score.goToPositionBottom(constant.getHeightBottom(),0.5);
                            clawMoveDownToggle.set(false);

                        }
                    }

                     */


                    }
                }
            }
        };



        //thread for imu and sensors(if needed)

        imuThread = new Thread(){
            @Override
            public void run(){
                while(opModeIsActive()){

                    if(gamepad1.right_stick_button) {
                        //while(opModeisActive(){
                        // Orientation angle = imu.getAngularOrientation();

                        //synchronized Orientation getAngularOrientation();
                        // }

                        if(imu.getAngularOrientation().firstAngle < 0){
                            isTurning = true;
                            drive.turn180(Math.PI/1.1);
                            isTurning = false;
                        }else{
                            isTurning = true;
                            drive.turn180(-Math.PI/1.1);
                            isTurning = false;
                        }
                    }
                    //turn scoring
                    if (gamepad1.left_bumper) {
                        if(imu.getAngularOrientation().firstAngle < 0){
                            isTurning = true;
                            drive.turn180(Math.PI/1.1);
                            isTurning = false;
                        }else{
                            isTurning = true;
                            drive.turn180(-Math.PI/1.1);
                            isTurning = false;
                        }

                    }

                }

            }
        };


        score.setClawPosition(constant.getClawOpenPos());
        score.setScoreBoolean(true, false, false);
        score.setCamPosition(constant.getStrafeLowCamPos());
        telemetry.addData("status", "initialized");
        telemetry.update();
        waitForStart();
        liftThread.start();
        imuThread.start();
        score.goToPosition(constant.getHeightBottom(), 0.6);
        score.setPower(constant.getSteadyPow());
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
            if(!isTurning) {
                if (gamepad1.left_stick_button) { // replace this with a button for sprint
                    drive.setPower(new Vector2D(gamepadX * SPRINT_LINEAR_MODIFIER, gamepadY * SPRINT_LINEAR_MODIFIER), gamepad1.right_stick_x * SPRINT_ROTATIONAL_MODIFIER, false);
                } else {
                    if (score.getEncoderPosition() > 480) {
                        drive.setPower(new Vector2D(gamepadX * 0.55, gamepadY * 0.55), gamepad1.right_stick_x * 0.28, false);
                    } else {
                        drive.setPower(new Vector2D(gamepadX * NORMAL_LINEAR_MODIFIER, gamepadY * NORMAL_LINEAR_MODIFIER), gamepad1.right_stick_x * NORMAL_ROTATIONAL_MODIFIER, false);
                    }
                }
            }
            if(gamepad1.right_bumper){
                drive.absTurnPID(Math.PI /2);
            }
            /*
            //turn scoring(turn in other thread)
            if (gamepad1.left_bumper) {
                if(score.scoreHigh()){
                    //high cone Max limit is 1370
                    steadyPower = false;
                    isTurnLifting = true;
                    score.goToPositionPID(constant.getHeightHigh(), 1);
                    steadyPower = true;
                    isTurnLifting = false;
                    score.setPower(constant.getSteadyPow());
                }else if(score.scoreMid()){
                    //medium cone
                    steadyPower = false;
                    isTurnLifting = true;
                    score.goToPositionPID(constant.getHeightMed(), 1);
                    steadyPower = true;
                    isTurnLifting = false;
                    score.setPower(constant.getSteadyPow());
                }else if(score.scoreLow()){
                    //low cone
                    steadyPower = false;
                    isTurnLifting = true;
                    score.goToPositionPID(constant.getHeightLow(), 0.8);
                    steadyPower = true;
                    isTurnLifting = false;
                    score.setPower(constant.getSteadyPow());
                }
            }


             */

            //Sets the booleans for which pole to go to for lifts system
            if (gamepad1.a) {
                //low cone
                score.setScoreBoolean(false,false,true);
            }
            //moves slides to medium pole height
            if(gamepad1.x){
                //medium cone
                score.setScoreBoolean(false, true, false);
            }
            //moves slides to low pole
            if(gamepad1.y){
                //high cone
                score.setScoreBoolean(true, false, false);
            }


            /*
            if (gamepad1.right_bumper && uprighterToggle) {
                //cone uprighter
                if (constant.getHeightBottom() == 0) {
                    constant.setHeightBottom(47);
                    score.goToBottom(constant.getHeightBottom(), 0.45);
                    uprighting = false;
                } else if (constant.getHeightBottom() == 47) {
                    constant.setHeightBottom(0);
                    score.goToBottom(constant.getHeightBottom(), 0.45);
                    uprighting = true;
                }
                uprighterToggle = false;

            }else if(!gamepad1.right_bumper){
                uprighterToggle = true;
            }

             */
            /*
            if (gamepad1.right_bumper && uprighterToggle) {
                //cone uprighter
                score.goToPosition(0, 0.45);
                constant.setClawClosePos(0.7);
                uprighterToggle = false;

            }else if(!gamepad1.right_bumper) {
                uprighterToggle = true;
            }

             */

            if(gamepad1.right_bumper){
                drive.absTurnPID(Math.PI / 2);
            }


            //resets the drive motor encoders
            if (gamepad1.share) {
                drive.resetIMU();
            }

            //TODO: add telemtry for gamepad a and y positions when you press them
            telemetry.addData("fl Pos", drive.getFLPosition());
            telemetry.addData("frPos", drive.getFRPosition());
            telemetry.addData("blPos", drive.getBLPosition());
            telemetry.addData("brPos", drive.getBRPosition());
            telemetry.addData("current angle", imu.getAngularOrientation().firstAngle);
            telemetry.addData("liftPos", score.getEncoderPosition());
            telemetry.addData("clawPos", score.getClawPosition());
            telemetry.addData("LeftliftPow", score.getLeftPower());
            telemetry.addData("RightliftPow", score.getRightPower());
            telemetry.addData("clawOpenCloseToggle", clawOpenCloseToggle.get());
            telemetry.addData("imu", imu.getAngularOrientation().firstAngle);
            //// telemetry.addData("blue", color.currentBlueColor());
            //telemetry.addData("red", color.currentRedColor());
            //  telemetry.update();
            //telemetry.update();
            telemetry.update();
            //loggingString += "FlPower: " + drive.getPowerFl() +" FrPower: " + drive.getPowerFr() + " BlPower: " + drive.getPowerBl() + " BrPower: " + drive.getPowerBr() + "\n";
            // loggingString += "CurrentAngle: " + imu.getAngularOrientation().firstAngle + "\n";


        }
        //drive.writeLoggerToFile();
        score.writeLoggerToFile();
        //score.writeLoggerToFile(loggingFile, loggingString);
        drive.setPower(0, 0, 0, 0);
        score.setPower(0);
        score.setClawPosition(1);
    }
}