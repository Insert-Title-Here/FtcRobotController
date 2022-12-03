package org.firstinspires.ftc.teamcode.League1.Testing.AutonomousTesting;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.League1.Autonomous.Vision.KevinGodPipeline;
import org.firstinspires.ftc.teamcode.League1.Common.Constants;
import org.firstinspires.ftc.teamcode.League1.Subsystems.MecDrive;
import org.firstinspires.ftc.teamcode.League1.Subsystems.ScoringSystem2;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.concurrent.atomic.AtomicBoolean;

@Disabled
@Autonomous
public class SegmentedDuplicateAuto extends LinearOpMode {
    MecDrive drive;
    ScoringSystem2 score;
    Constants constants;
    Thread armThread, feedForward, idController;
    ElapsedTime time = new ElapsedTime();
    AtomicBoolean hold, armUp, armDown, finalMove, linkageUp;

    ColorRangeSensor distance, color;
    Servo cameraServo;

    OpenCvWebcam camera;
    KevinGodPipeline pipeline;
    KevinGodPipeline.ParkPos parkPos;

    int normalizeDistance;
    boolean failed;


    @Override
    public void runOpMode() throws InterruptedException {
        distance = hardwareMap.get(ColorRangeSensor.class, "distance");
        color = hardwareMap.get(ColorRangeSensor.class, "color");

        drive = new MecDrive(hardwareMap, false, telemetry, color);
        constants = new Constants();
        score = new ScoringSystem2(hardwareMap, constants);
        hold = new AtomicBoolean(false);
        armUp = new AtomicBoolean(false);
        finalMove = new AtomicBoolean(false);
        linkageUp = new AtomicBoolean(false);
        armDown = new AtomicBoolean(false);

        score.setLinkagePosition(Constants.linkageDown);
        score.setGrabberPosition(constants.grabbing);


        cameraServo = hardwareMap.get(Servo.class, "camera");
        failed = false;


        distance.setGain(300);


        /*
        idController = new Thread(){
            @Override
            public void run() {



                while(opModeIsActive()){
                    if(hold.get()){
                        ElapsedTime time = new ElapsedTime();
                        double startTime = time.milliseconds();

                        int leftIntegralSum = 0;
                        int rightIntegralSum = 0;


                        int rLiftPos = score.getRightEncoderPos();
                        int lLiftPos = -1 * score.getLeftEncoderPos();

                        int tics = score.getHeight();

                        int leftPreviousError = Math.abs(tics - lLiftPos);
                        int rightPreviousError = Math.abs(tics - rLiftPos);

                        while(hold.get()){

                            rLiftPos = score.getRightEncoderPos();
                            lLiftPos = -1 * score.getLeftEncoderPos();

                            double currentTime = time.milliseconds();

                            int leftError = tics - lLiftPos;
                            int rightError = tics - rLiftPos;

                            leftIntegralSum += (0.5 * (leftError + leftPreviousError) * (currentTime - startTime));
                            rightIntegralSum += (0.5 * (rightError + rightPreviousError) * (currentTime - startTime));

                            double leftDerivative = (leftError - leftPreviousError)/(currentTime - startTime);
                            double rightDerivative = (rightError - rightPreviousError)/(currentTime - startTime);

                            double leftPower = (pid.i * leftIntegralSum) + (pid.d * leftDerivative);
                            double rightPower = (pid.i * rightIntegralSum) + (pid.d * rightDerivative);

                            if(tics < ((rLiftPos + lLiftPos) / 2)){
                                leftPower *= -1;
                                rightPower *= -1;
                            }

                            score.setPower(rightPower, leftPower);


                            startTime = currentTime;
                            leftPreviousError = leftError;
                            rightPreviousError = rightError;



                        }




                    }
                }
            }
        };

         */

        armThread = new Thread() {
            @Override
            public void run() {
                //score.setLinkagePosition(0.7);
                while (opModeIsActive()) {
                    if (finalMove.get()) {

                        score.setLinkagePositionLogistic(Constants.linkageUp, 250);
                        finalMove.set(false);

                    } else if (armUp.get()) {
                        try {
                            Thread.sleep(100);
                        } catch (InterruptedException e) {
                            e.printStackTrace();
                        }
                        score.setLinkagePosition(0.63);
                        armUp.set(false);

                    }else if(linkageUp.get()){
                        try {
                            sleep(200);
                        } catch (InterruptedException e) {
                            e.printStackTrace();
                        }
                        score.setLinkagePosition(Constants.linkageUp);
                        linkageUp.set(false);
                    }


                }

                //Might need this
                //hold.set(true);
            }
        };


        feedForward = new Thread() {
            @Override
            public void run() {
                while (opModeIsActive()) {
                    if (hold.get()) {
                        score.setPower(0.2);
                    }
                }
            }
        };


        WebcamName webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);
        pipeline = new KevinGodPipeline(telemetry, drive, KevinGodPipeline.AutoSide.BLUE_RIGHT);

        camera.setPipeline(pipeline);


        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(320, 176, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addData("Camera Init Error", errorCode);
                telemetry.update();

            }
        });

        FtcDashboard.getInstance().startCameraStream(camera, 0);


        cameraServo.setPosition(Constants.sleeve);


        waitForStart();
        double startTime = time.seconds();



        armThread.start();
        feedForward.start();
        parkPos = pipeline.getPosition();
        cameraServo.setPosition(Constants.pole);
        pipeline.changeMode(KevinGodPipeline.Mode.POLE);



        score.setLinkagePosition(0.8);
        sleep(200);
        drive.goTOPIDPos(-2050, 0.5,MecDrive.MovementType.STRAIGHT);
        sleep(100);

        score.setLinkagePosition(Constants.linkageUp);
        sleep(200);

        hold.set(false);
        score.moveToPosition(830, 1);
        hold.set(true);
        drive.tankRotatePID(Math.PI / 5.3, 0.7, true);
        sleep(100);

        //sleep(200);
        //drive.simpleMoveToPosition(-250, MecDrive.MovementType.ROTATE, 0.4);
        normalizeDistance = pipeline.normalize(0.2, 172, 2);

        score.setLinkagePositionLogistic(Constants.linkageScore, 100, 50);

        sleep(500);
        score.setGrabberPosition(0.3);
        sleep(200);

        score.setLinkagePositionLogistic(Constants.linkageUp, 100, 50);

        hold.set(false);
        score.moveToPosition(0, 0.5);





        pipeline.changeMode(KevinGodPipeline.Mode.REDCONE);
        drive.tankRotatePID(Math.PI / 2, 0.85, false);
        score.setGrabberPosition(0.65);


        for (int i = 0; i < 4; i++) {


            score.setLinkagePosition(constants.linkageUp);

            pipeline.normalizeStrafe(0.3, 165, 5);

            if (pipeline.getNormalizationBroke()) {
                drive.tankRotatePID(Math.PI / 2, 0.6, false);
                break;
            }


            score.setLinkagePosition(0.75 + (i * 0.04));

            double startDistanceTime = time.seconds();
            while (distance.getDistance(DistanceUnit.CM) > 3.8) {
                drive.setPowerAuto(0.42, MecDrive.MovementType.STRAIGHT);

                telemetry.addData("distance", distance.getDistance(DistanceUnit.CM));
                telemetry.update();

                if (time.seconds() - startDistanceTime > 3) {
                    drive.simpleBrake();
                    drive.tankRotatePID(Math.PI / 2, 0.6, false);
                    failed = true;
                    break;
                }

            }

            if (failed) {
                break;
            }

            drive.simpleBrake();


            score.setGrabberPosition(constants.grabbing);
            sleep(400);


            //score.moveToPosition(200, 1);

            if (i >= 2) {
                drive.simpleMoveToPosition(-10, MecDrive.MovementType.STRAIGHT, 0.3);

            }

            score.setLinkagePosition(0.63);
            //hold.set(true);
            sleep(300);


            pipeline.changeMode(KevinGodPipeline.Mode.POLE);
            cameraServo.setPosition(Constants.pole);

            drive.goTOPIDPos(-1035, 1, MecDrive.MovementType.STRAIGHT);
            if (time.seconds() - startTime > 26) {
                break;
            }

            finalMove.set(true);
            sleep(300);

            //score.moveToPosition(0, 1);

            hold.set(false);
            score.moveToPosition(830, 1);
            hold.set(true);


            //TODO: see if want to change to Math.PI/3.7
            drive.tankRotatePID(Math.PI / 5, 0.8, true);
            //drive.simpleMoveToPosition(290, MecDrive.MovementType.ROTATE, 0.4);
            normalizeDistance = pipeline.normalize(0.2, 172, 2);

            score.setLinkagePositionLogistic(Constants.linkageScore, 300, 50);


            sleep(400);
            score.setGrabberPosition(0.3);
            sleep(200);


            if (pipeline.getNormalizationBroke()) {
                drive.tankRotatePID(Math.PI / 2, 0.6, false);
                break;
            }


            //drive.simpleMoveToPosition(-30, MecDrive.MovementType.STRAIGHT, 0.3);


            cameraServo.setPosition(Constants.cone);
            sleep(200);


            //score.setLinkagePositionLogistic(Constants.linkageDown, 250, 30);
            pipeline.changeMode(KevinGodPipeline.Mode.REDCONE);

            score.setLinkagePosition(Constants.linkageUp);
            sleep(100);


            //drive.simpleMoveToPosition(70, MecDrive.MovementType.STRAIGHT, 0.4);

            drive.tankRotatePID(Math.PI / 2, 0.85, false);

            hold.set(false);
            sleep(300);
            score.moveToPosition(0, 0.5);
            //drive.simpleMoveToPosition(-320 + normalizeDistance, MecDrive.MovementType.ROTATE, 0.4);
            //drive.tankRotatePID(Math.PI/2, 1);            //pipeline.normalizeToPole(0.3, 42, 5);


            //drive.simpleMoveToPosition(150, MecDrive.MovementType.STRAFE, 0.4);
            //drive.goTOPIDPos(150, 1, MecDrive.MovementType.STRAIGHT);

            score.setGrabberPosition(0.65);


        }

        score.setGrabberPosition(constants.grabbing);

        camera.closeCameraDevice();


        finalMove.set(true);


        if (failed) {
            if (parkPos == KevinGodPipeline.ParkPos.CENTER) {
                drive.simpleMoveToPosition(-700, MecDrive.MovementType.STRAIGHT, 0.5);

            } else if (parkPos == KevinGodPipeline.ParkPos.RIGHT) {
                drive.simpleMoveToPosition(-1500, MecDrive.MovementType.STRAIGHT, 0.5);

            }

        } else {
            if (parkPos == KevinGodPipeline.ParkPos.LEFT) {
                drive.simpleMoveToPosition(-500, MecDrive.MovementType.STRAIGHT, 0.8);

            } else if (parkPos == KevinGodPipeline.ParkPos.RIGHT) {
                drive.simpleMoveToPosition(700, MecDrive.MovementType.STRAIGHT, 1);

            }
        }

        //Will have to check if this aligns straight already (need color sensor or not) ->
        // may need to turn into slight diagonal instead of straight to check color
        //drive.simpleMoveToPosition(675, MecDrive.MovementType.STRAIGHT, 0.3);


    }


}