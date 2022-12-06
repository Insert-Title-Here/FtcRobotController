package org.firstinspires.ftc.teamcode.V2.Autonomous;

//import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.League1.Autonomous.Vision.KevinGodPipeline;
import org.firstinspires.ftc.teamcode.League1.Autonomous.Vision.KevinGodPipelineV2;
import org.firstinspires.ftc.teamcode.League1.Common.Constants;
import org.firstinspires.ftc.teamcode.League1.Subsystems.MecDrive;
import org.firstinspires.ftc.teamcode.League1.Subsystems.ScoringSystem2;
import org.firstinspires.ftc.teamcode.V2.NewSubsystem.ScoringSystemV2;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.concurrent.atomic.AtomicBoolean;

@Disabled
@Autonomous
public class V2AutoPerfectBlueRight extends LinearOpMode {
    MecDrive drive;
    ScoringSystemV2 score;
    Constants constants;
    Thread armThread, feedForward, idController;
    ElapsedTime time = new ElapsedTime();
    AtomicBoolean hold, armUp, armDown, finalMove, linkageUp;
    int cycles;

    ColorRangeSensor distance, color;
    Servo cameraServo;

    OpenCvWebcam camera;
    KevinGodPipelineV2 pipeline;
    KevinGodPipelineV2.ParkPos parkPos;

    int normalizeDistance;
    boolean failed;
    boolean preloadSuccess = false;


    @Override
    public void runOpMode() throws InterruptedException {
        distance = hardwareMap.get(ColorRangeSensor.class, "distance");
        color = hardwareMap.get(ColorRangeSensor.class, "color");

        drive = new MecDrive(hardwareMap, false, telemetry, color);
        constants = new Constants();
        score = new ScoringSystemV2(hardwareMap, constants);
        hold = new AtomicBoolean(false);
        armUp = new AtomicBoolean(false);
        finalMove = new AtomicBoolean(false);
        linkageUp = new AtomicBoolean(false);
        armDown = new AtomicBoolean(false);

        score.setLinkagePosition(Constants.linkageDownV2);
        score.setGrabberPosition(constants.grabbing);


        cameraServo = hardwareMap.get(Servo.class, "camera");
        failed = false;


        distance.setGain(300);

        cycles = 5;


        armThread = new Thread() {
            @Override
            public void run() {
                //score.setLinkagePosition(0.7);
                while (opModeIsActive()) {
                    if (armUp.get()) {
                        hold.set(false);
                        score.setLinkagePosition(Constants.linkageScoreV2 - 0.02);

                        score.moveToPosition(1400, 1);
                        hold.set(true);


                        try {
                            Thread.sleep(500);
                        } catch (InterruptedException e) {
                            e.printStackTrace();
                        }
                        score.setGrabberPosition(0.3);
                        try {
                            Thread.sleep(200);
                        } catch (InterruptedException e) {
                            e.printStackTrace();
                        }
                        armUp.set(false);
                    } else if (armDown.get()) {
                        hold.set(false);
                        score.setLinkagePosition(Constants.linkageUpV2);
                        score.moveToPosition(0, 0.5);
                        //score.setLinkagePositionLogistic(Constants.linkageDown, 250, 30);
                        armDown.set(false);
                    } else if (finalMove.get()) {

                        score.setLinkagePositionLogistic(Constants.linkageUpV2, 100);
                        finalMove.set(false);

                    } else if (linkageUp.get()) {
                        try {
                            sleep(200);
                        } catch (InterruptedException e) {
                            e.printStackTrace();
                        }
                        score.setLinkagePosition(Constants.linkageUpV2);
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
        pipeline = new KevinGodPipelineV2(telemetry, drive, KevinGodPipelineV2.AutoSide.RED_RIGHT);

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

        //FtcDashboard.getInstance().startCameraStream(camera, 0);


        cameraServo.setPosition(Constants.sleeveV2);



        waitForStart();

        double startTime = time.seconds();

        parkPos = pipeline.getPosition();

        if(parkPos == KevinGodPipelineV2.ParkPos.LEFT){
            cycles = 4;
        }

        //parkPos = KevinGodPipelineV2.ParkPos.LEFT;
        //pipeline.changeMode(KevinGodPipelineV2.Mode.POLE);


        armThread.start();
        feedForward.start();


        pipeline.changeMode(KevinGodPipelineV2.Mode.POLE);
        cameraServo.setPosition(Constants.poleV2);


        linkageUp.set(true);
        drive.simpleMoveToPosition(-1580, MecDrive.MovementType.STRAIGHT, 0.85);

        sleep(100);

        drive.tankRotatePID(Math.PI / 2, 1, false);

        drive.simpleMoveToPosition(670, MecDrive.MovementType.STRAIGHT, 0.6);

        drive.tankRotatePID(3 * Math.PI / 8, 1, false);

        drive.simpleMoveToPosition(-30, MecDrive.MovementType.STRAFE, 0.5);

        drive.simpleMoveToPosition(-20, MecDrive.MovementType.ROTATE, 0.5);


        pipeline.normalize(0.15, 159, 3);


        //drive.simpleMoveToPosition(50, MecDrive.MovementType.STRAIGHT, 0.3);

        if(distance.getNormalizedColors().blue > 0.85) {

            hold.set(false);
            score.moveToPosition(1340, 1, 1.2);
            hold.set(true);



            score.setLinkagePositionLogistic(0.8, 100);

            sleep(50);

            score.setGrabberPosition(Constants.score);

            sleep(100);

            score.setLinkagePositionLogistic(0.245, 100);

            score.setGrabberPosition(Constants.openV2 - 0.03);

            hold.set(false);
            score.moveToPosition(0, 0.8);

            preloadSuccess = true;
        } else {
            cycles = 5;
            drive.simpleMoveToPosition(-50, MecDrive.MovementType.STRAIGHT, 0.5);
            score.setGrabberPosition(Constants.openV2 - 0.03);
            score.setLinkagePositionLogistic(0.245, 100);
        }

        for (int i = 0; i < cycles; i++) {



            if(i == 0 && preloadSuccess) {

                double startDistanceTime = time.seconds();
                while (distance.getDistance(DistanceUnit.CM) > 3.5) {
                    drive.setPowerAuto(0.2, MecDrive.MovementType.STRAIGHT);

                    telemetry.addData("distance", distance.getDistance(DistanceUnit.CM));
                    telemetry.update();

                    /*if (time.seconds() - startDistanceTime > 3) {
                        drive.simpleBrake();
                        drive.tankRotatePID(Math.PI / 2, 0.6, false);
                        failed = true;
                        break;
                    }*/

                }




                drive.simpleBrake();

            } else {
                drive.simpleMoveToPosition(10, MecDrive.MovementType.STRAIGHT, 0.5);
            }



            score.setGrabberPosition(Constants.grabbing);

            sleep(70);

            score.setLinkagePositionLogistic(Constants.linkageUpV2, 100);

            //drive.simpleMoveToPosition(-distanceDriven, MecDrive.MovementType.STRAIGHT, 0.4);

            pipeline.normalize(0.15, 159, 3);

            hold.set(false);

            score.moveToPosition(1360, 1, 1.2);

            hold.set(false);


            score.setLinkagePositionLogistic(0.8, 100);

            sleep(50);

            score.setGrabberPosition(Constants.score);

            sleep(100);

            score.setLinkagePositionLogistic(0.245 - ((i + 1) * 0.03), 100);
            score.setGrabberPosition(Constants.openV2 - 0.03);



            hold.set(false);

            score.moveToPosition(0, 0.8);



            if (time.seconds() - startTime > 26) {
                i = 5;
            }

        }

        drive.simpleMoveToPosition(-160, MecDrive.MovementType.ROTATE, 0.5);

        sleep(50);

        if (parkPos == KevinGodPipelineV2.ParkPos.CENTER) {
            drive.simpleMoveToPosition(-730, MecDrive.MovementType.STRAIGHT, 1);
        } else if (parkPos == KevinGodPipelineV2.ParkPos.LEFT) {
            drive.simpleMoveToPosition(-1500, MecDrive.MovementType.STRAIGHT, 1);
        }

        score.setGrabberPosition(Constants.grabbing);

        sleep(50);

    }
}



