package org.firstinspires.ftc.teamcode.V2.Autonomous.State;

//import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.AprilTagsTesting.KevinGodPipelineAprilTag;
import org.firstinspires.ftc.teamcode.League1.Common.Constants;
import org.firstinspires.ftc.teamcode.League1.Subsystems.MecDrive;
import org.firstinspires.ftc.teamcode.V2.NewSubsystem.MecDriveV2;
import org.firstinspires.ftc.teamcode.V2.NewSubsystem.ScoringSystemV2EpicLift;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.concurrent.atomic.AtomicBoolean;

@Autonomous
public class BlueRightStateThreadlessKrishTesting extends LinearOpMode {
    MecDriveV2 drive;
    ScoringSystemV2EpicLift score;
    Constants constants;
    Thread armThread, feedForward, idController;
    ElapsedTime time = new ElapsedTime();
    AtomicBoolean hold, armUp, armDown, finalMove, linkageUp;
    int cycles;
    int liftPos = 900;

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

        drive = new MecDriveV2(hardwareMap, false, telemetry, true);
        constants = new Constants();
        score = new ScoringSystemV2EpicLift(hardwareMap, constants, telemetry);
        hold = new AtomicBoolean(false);
        armUp = new AtomicBoolean(false);
        finalMove = new AtomicBoolean(false);
        linkageUp = new AtomicBoolean(false);
        armDown = new AtomicBoolean(false);

        score.setLinkagePosition(Constants.linkageScoreV2 - 0.07);
        score.setGrabberPosition(constants.grabbing);


        cameraServo = hardwareMap.get(Servo.class, "camera");
        failed = false;


        distance.setGain(300);

        cycles = 5;


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


        //linkageUp.set(true);
        //drive.simpleMoveToPosition(-1700, MecDrive.MovementType.STRAIGHT, 0.55);
        drive.goTOPIDPosWithRampUp(-2200, 1, MecDriveV2.MovementType.STRAIGHT, 0.85);

        sleep(100);

        drive.tankRotatePID(Math.PI / 2, 1, false);

        drive.simpleMoveToPosition(675, MecDriveV2.MovementType.STRAIGHT, 0.4);

        drive.tankRotatePID(3 * Math.PI / 8, 1, false);

        pipeline.normalizeStrafe(0.3, 150, 2);

        pipeline.changeMode(KevinGodPipelineAprilTag.Mode.POLE);
        cameraServo.setPosition(Constants.poleV2);

        sleep(600);

        pipeline.normalize(0.2, 155, 2);

        if(distance.getNormalizedColors().blue > 0.65) {

            score.setLinkagePositionLogistic(Constants.linkageUpV2Auto, 300, 100);

            hold.set(false);
            score.newLiftPID(1000, 0.75);
            hold.set(true);


            //sleep(350);

            score.setLinkagePositionLogistic(0.8, 100);

            sleep(200);

            score.setGrabberPosition(Constants.score);

            sleep(200);

            score.setLinkagePositionLogistic(0.245, 100);

            score.setGrabberPosition(Constants.openV2);

            hold.set(false);
            score.moveToPosition(0, 0.63);
            hold.set(true);

            preloadSuccess = true;
        } else {
            pipeline.normalize(0.2, 155, 3);
            cycles = 5;
            drive.simpleMoveToPosition(-63, MecDriveV2.MovementType.STRAIGHT, 0.5);
            score.setGrabberPosition(Constants.openV2);
            score.setLinkagePositionLogistic(0.245, 100);
        }

        for (int i = 0; i < cycles; i++) {



            if(i == 0) {

                /*double startDistanceTime = time.seconds();
                while (distance.getDistance(DistanceUnit.CM) > 6.5) {
                    drive.setPowerAuto(0.2, MecDrive.MovementType.STRAIGHT);

                    telemetry.addData("distance", distance.getDistance(DistanceUnit.CM));
                    telemetry.update();

                    if (time.seconds() - startDistanceTime > 3) {
                        drive.simpleBrake();
                        score.setLinkagePosition(Constants.linkageUpV2);
                        drive.tankRotatePID(Math.PI / 2, 0.6, false);
                        failed = true;
                        break;
                    }

                }

                if (failed) {
                    break;
                }




                drive.simpleBrake();
*/

                drive.simpleMoveToPosition(75, MecDriveV2.MovementType.STRAIGHT, 0.5);

            } else {
                if(i < 3) {
                    drive.simpleMoveToPosition(8, MecDriveV2.MovementType.STRAIGHT, 0.5);
                }
            }

            if (failed == true) {
                break;
            }


            score.setGrabberPosition(Constants.grabbing);

            sleep(100);

            score.setLinkagePositionLogistic(Constants.linkageUpV2Auto, 300, 100);

            //drive.simpleMoveToPosition(-distanceDriven, MecDrive.MovementType.STRAIGHT, 0.4);

            pipeline.normalize(0.2, 155, 3);

            hold.set(false);
            if (true) {
                score.newLiftPID(1000, 0.75);
            } else {
                score.newLiftPID(liftPos - 80, 0.75);
            }
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
            score.moveToPosition(0, 0.63);
            hold.set(true);

            if (time.seconds() - startTime > 25) {
                i = 5;
            }

        }

        if (!failed) {
            drive.simpleMoveToPosition(-160, MecDriveV2.MovementType.ROTATE, 0.5);
        } else {
            drive.tankRotatePID(Math.PI / 2, 1, false);
        }

        sleep(50);

        if (parkPos == KevinGodPipelineAprilTag.ParkPos.CENTER) {
            drive.simpleMoveToPosition(-700, MecDriveV2.MovementType.STRAIGHT, 1);
        } else if (parkPos == KevinGodPipelineAprilTag.ParkPos.LEFT) {
            drive.simpleMoveToPosition(-1450, MecDriveV2.MovementType.STRAIGHT, 1);
        }

        score.setGrabberPosition(Constants.grabbing);

        sleep(500);


    }
}



