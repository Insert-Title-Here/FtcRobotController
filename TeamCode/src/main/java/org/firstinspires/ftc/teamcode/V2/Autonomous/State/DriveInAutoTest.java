package org.firstinspires.ftc.teamcode.V2.Autonomous.State;

//import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.AprilTagsTesting.KevinGodPipelineAprilTag;
import org.firstinspires.ftc.teamcode.League1.Autonomous.Vision.KevinGodPipelineV2;
import org.firstinspires.ftc.teamcode.League1.Autonomous.Vision.KevinGodPipelineV2Comp;
import org.firstinspires.ftc.teamcode.League1.Common.Constants;
import org.firstinspires.ftc.teamcode.V2.NewSubsystem.MecDriveV2;
import org.firstinspires.ftc.teamcode.V2.NewSubsystem.ScoringSystemV2;
import org.firstinspires.ftc.teamcode.V2.NewSubsystem.ScoringSystemV2EpicLift;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.concurrent.atomic.AtomicBoolean;

@Autonomous
public class DriveInAutoTest extends LinearOpMode {
    MecDriveV2 drive;
    ScoringSystemV2EpicLift score;
    //Constants constants;
    Thread armThread, feedForward, idController;
    ElapsedTime time = new ElapsedTime();
    AtomicBoolean hold, armUp, armDown, finalMove, linkageUp;
    int cycles;

    ColorRangeSensor distance, color;
    Servo cameraServo;

    OpenCvWebcam camera;
    KevinGodPipelineAprilTag pipeline;
    KevinGodPipelineAprilTag.ParkPos parkPos;

    int normalizeDistance;
    boolean failed;
    boolean preloadSuccess = false;


    @Override
    public void runOpMode() throws InterruptedException {
        distance = hardwareMap.get(ColorRangeSensor.class, "distance");

        drive = new MecDriveV2(hardwareMap, false, telemetry);
        //constants = newConstants();
        score = new ScoringSystemV2EpicLift(hardwareMap, telemetry);
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
                        score.moveToPosition(1350, 0.85);
                        hold.set(true);

                        armUp.set(false);
                    } else if (armDown.get()) {
                        hold.set(false);
                        score.moveToPosition(0, 0.8);
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
                        score.setPowerSingular(0.2);
                    }
                }
            }
        };


        WebcamName webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);
        pipeline = new KevinGodPipelineAprilTag(telemetry, drive, KevinGodPipelineAprilTag.AutoSide.BLUE_RIGHT, true);

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

        pipeline.changeMode(KevinGodPipelineAprilTag.Mode.BLUECONE);
        cameraServo.setPosition(Constants.coneV2);

        parkPos = pipeline.getPosition();

        if(parkPos == KevinGodPipelineAprilTag.ParkPos.LEFT){
            //cycles = 4;
        }

        //parkPos = KevinGodPipelineV2.ParkPos.LEFT;
        //pipeline.changeMode(KevinGodPipelineV2.Mode.POLE);


        armThread.start();
        feedForward.start();


        //linkageUp.set(true);
        drive.simpleMoveToPosition(-1700, MecDriveV2.MovementType.STRAIGHT, 0.55);

        sleep(100);

        drive.tankRotatePID(Math.PI / 2, 1, false);

        drive.simpleMoveToPosition(710, MecDriveV2.MovementType.STRAIGHT, 0.4);

        drive.tankRotatePID(3 * Math.PI / 8, 1, false);

        pipeline.normalizeStrafe(0.3, 150, 2);

        pipeline.changeMode(KevinGodPipelineAprilTag.Mode.POLE);
        cameraServo.setPosition(Constants.poleV2);

        sleep(600);

        pipeline.normalize(0.2, 155, 2);

        if(distance.getNormalizedColors().blue > 0.65) {

            score.setLinkagePositionLogistic(Constants.linkageUpV2Auto, 300, 100);

            hold.set(false);
            score.moveToPosition(750, 1);
            int liftPos = score.moveToPosition(1000, 0.4, 10, true);
            score.moveToPosition(liftPos + 10, 1);
            hold.set(true);


            //sleep(350);

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
            pipeline.normalize(0.2, 155, 3);
            cycles = 5;
            drive.simpleMoveToPosition(-63, MecDriveV2.MovementType.STRAIGHT, 0.5);
            score.setGrabberPosition(Constants.openV2);
            score.setLinkagePositionLogistic(0.245, 100);
        }




    }
}



