package org.firstinspires.ftc.teamcode.Competition.State.Autonomous.OldAutonomous;

//import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Competition.Interleagues.Autonomous.Vision.UsedPipeline.KevinGodPipelineV2Comp;
import org.firstinspires.ftc.teamcode.Competition.Interleagues.Common.Constants;
import org.firstinspires.ftc.teamcode.Competition.Interleagues.Subsystems.Used.MecDrive;
import org.firstinspires.ftc.teamcode.Competition.State.Subsystems.Current.ScoringSystemV2EpicLift;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.concurrent.atomic.AtomicBoolean;
@Disabled

@Autonomous(name="Red Right (Field 2)")
public class V2RedRightField2 extends LinearOpMode {
    MecDrive drive;
    ScoringSystemV2EpicLift score;
    //Constants constants;
    Thread armThread, feedForward, idController;
    ElapsedTime time = new ElapsedTime();
    AtomicBoolean hold, armUp, armDown, finalMove, linkageUp;
    int cycles;

    ColorRangeSensor distance, color;
    Servo cameraServo;

    OpenCvWebcam camera;
    KevinGodPipelineV2Comp pipeline;
    KevinGodPipelineV2Comp.ParkPos parkPos;

    int normalizeDistance;
    boolean failed;
    boolean preloadSuccess = false;


    @Override
    public void runOpMode() throws InterruptedException {
        distance = hardwareMap.get(ColorRangeSensor.class, "distance");
        //color = hardwareMap.get(ColorRangeSensor.class, "color");

        drive = new MecDrive(hardwareMap, false, telemetry, true);
        //constants = newConstants();
        score = new ScoringSystemV2EpicLift(hardwareMap, telemetry, false);
        hold = new AtomicBoolean(false);
        armUp = new AtomicBoolean(false);
        finalMove = new AtomicBoolean(false);
        linkageUp = new AtomicBoolean(false);
        armDown = new AtomicBoolean(false);

        score.setLinkagePosition(Constants.linkageScoreV2 - 0.07);
        score.setGrabberPosition(Constants.grabbing);


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
                        score.moveToPosition(1350, 1);
                        hold.set(true);

                        armUp.set(false);
                    } else if (armDown.get()) {
                        hold.set(false);
                        score.moveToPosition(0, 0.8);
                        //score.setLinkagePositionLogistic(Constants.linkageDown, 250, 30);
                        armDown.set(false);
                    } else if (finalMove.get()) {

                        score.setLinkagePositionLogistic(Constants.linkageUpV2, 1000);
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
                        score.setPowerSingular(0.2);
                    }
                }
            }
        };


        WebcamName webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);
        pipeline = new KevinGodPipelineV2Comp(telemetry, drive, KevinGodPipelineV2Comp.AutoSide.RED_RIGHT, false);

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

        if(parkPos == KevinGodPipelineV2Comp.ParkPos.LEFT){
            //cycles = 4;
        }

        //parkPos = KevinGodPipelineV2.ParkPos.LEFT;
        //pipeline.changeMode(KevinGodPipelineV2.Mode.POLE);


        armThread.start();
        feedForward.start();


        pipeline.changeMode(KevinGodPipelineV2Comp.Mode.POLE);
        cameraServo.setPosition(Constants.poleV2);


        //linkageUp.set(true);
        drive.simpleMoveToPosition(-1650, MecDrive.MovementType.STRAIGHT, 0.55);

        sleep(100);

        drive.tankRotatePID(Math.PI / 2, 1, false);

        drive.simpleMoveToPosition(710, MecDrive.MovementType.STRAIGHT, 0.4);

        drive.tankRotatePID(3 * Math.PI / 8, 1, false);

        drive.simpleMoveToPosition(-50, MecDrive.MovementType.STRAFE, 0.5);

        drive.simpleMoveToPosition(-10, MecDrive.MovementType.ROTATE, 0.3);

        if(distance.getNormalizedColors().red > 0.65) {

            pipeline.normalize(0.2, 159, 3);

            hold.set(false);
            score.moveToPosition(1230, 0.85);
            hold.set(true);


            sleep(350);

            score.setLinkagePositionLogistic(0.8, 100);

            sleep(200);

            score.setGrabberPosition(Constants.score);

            sleep(200);

            score.setLinkagePositionLogistic(0.245, 100);

            score.setGrabberPosition(Constants.openV2);

            hold.set(false);
            score.moveToPosition(0, 0.8);
            hold.set(true);

            preloadSuccess = true;
        } else {
            pipeline.normalize(0.2, 159, 3);
            cycles = 5;
            drive.simpleMoveToPosition(-50, MecDrive.MovementType.STRAIGHT, 0.5);
            score.setGrabberPosition(Constants.openV2);
            score.setLinkagePositionLogistic(0.245, 500, 100);
        }

        for (int i = 0; i < cycles; i++) {



            if(i == 0) {

                /*double startDistanceTime = time.seconds();
                while (distance.getDistance(DistanceUnit.CM) > 8) {
                    drive.setPowerAuto(0.2, MecDrive.MovementType.STRAIGHT);

                    telemetry.addData("distance", distance.getDistance(DistanceUnit.CM));
                    telemetry.update();

                    if (time.seconds() - startDistanceTime > 3) {
                        drive.simpleBrake();
                        score.setLinkagePosition(Constants.linkageUpV2);
                        drive.tankRotate(Math.PI / 2, 0.2);
                        failed = true;
                        break;
                    }

                }

                if (failed) {
                    break;
                }




                drive.simpleBrake();

                 */
                drive.simpleMoveToPosition(50, MecDrive.MovementType.STRAIGHT, 0.5);


            } else {

                if(i < 3) {
                    drive.simpleMoveToPosition(12, MecDrive.MovementType.STRAIGHT, 0.5);
                }
            }

            if (failed == true) {
                break;
            }


            score.setGrabberPosition(Constants.grabbing);

            sleep(100);

            score.setLinkagePositionLogistic(Constants.linkageUpV2Auto, 300, 100);

            //drive.simpleMoveToPosition(-distanceDriven, MecDrive.MovementType.STRAIGHT, 0.4);

            pipeline.normalize(0.2, 159, 3);

            hold.set(false);
            score.moveToPosition(1320, 0.85);
            hold.set(true);

            sleep(50);

            score.setLinkagePosition(0.8);

            sleep(100);

            score.setGrabberPosition(Constants.open);

            sleep(600);

            score.setLinkagePositionLogistic(0.245 - ((i + 1) * 0.03), 800, 100);
            score.setGrabberPosition(Constants.openV2);


            //sleep(250);

            hold.set(false);
            score.moveToPosition(0, 0.8);
            hold.set(true);

            if (time.seconds() - startTime > 25) {
                i = 5;
            }

        }

        if (!failed) {
            drive.simpleMoveToPosition(-160, MecDrive.MovementType.ROTATE, 0.5);
        } else {
            drive.tankRotatePID(Math.PI / 2, 1, false);
        }

        sleep(50);

        if (parkPos == KevinGodPipelineV2Comp.ParkPos.CENTER) {
            drive.simpleMoveToPosition(-700, MecDrive.MovementType.STRAIGHT, 1);
        } else if (parkPos == KevinGodPipelineV2Comp.ParkPos.LEFT) {
            drive.simpleMoveToPosition(-1450, MecDrive.MovementType.STRAIGHT, 1);
        }

        score.setGrabberPosition(Constants.grabbing);

        sleep(500);

    }
}



