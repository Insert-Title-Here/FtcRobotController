//red path two avoids team game element

package org.firstinspires.ftc.teamcode.Autonomous.TestMeAutos;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Common.Constants;
import org.firstinspires.ftc.teamcode.Common.MecDriveV2;
import org.firstinspires.ftc.teamcode.Common.ScoringSystem;
import org.firstinspires.ftc.teamcode.Testing.OpenCV.BarcodePipeline;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

//@Disabled
@Autonomous
public class RedFrontAutoTest extends LinearOpMode {

    MecDriveV2 drive;
    ScoringSystem score;
    ElapsedTime time;

    //Pipeline is a class made by Kevin to choose which auto to run based on TSE position
    BarcodePipeline pipeline;
    OpenCvWebcam camera;

    //The actual position being stored (changes each time)
    BarcodePipeline.BarcodePosition barcodePos;

    public void runOpMode() {

        time = new ElapsedTime();
        drive = new MecDriveV2(hardwareMap, false, telemetry, time);
        score = new ScoringSystem(hardwareMap, telemetry, time);
        score.setGrabberPosition(Constants.GRABBING);
        score.setBumperPixelRelease(Constants.AUTO_SCORING_CLAMP_OPEN);
        score.setIntakeLiftPos(Constants.INTAKE_LINKAGE_DOWN-0.1);

        //Vision stuff to assign 1, 2, or 3 to rando
        //Camera initialization
        //Mapping camera to the code
        WebcamName webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        //Create a webcam + pipeline and assign the pipeline to the webcam
        camera = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);
        pipeline = new BarcodePipeline(telemetry);
        camera.setPipeline(pipeline);

        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(320, 240 , OpenCvCameraRotation.UPRIGHT);
            }
            @Override
            public void onError(int errorCode) {
                telemetry.addData("Camera Init Error", errorCode);
                telemetry.update();
            }
        });
        FtcDashboard.getInstance().startCameraStream(camera, 0);

        waitForStart();
        barcodePos = pipeline.getPos();


        score.setIntakeLiftPos(Constants.INTAKE_LINKAGE_UP);
        score.setBumperPixelRelease(Constants.AUTO_SCORING_CLAMP_CLOSED);
        sleep(1000);

        if (barcodePos == BarcodePipeline.BarcodePosition.LEFT) {
             random1();
        }
        else if (barcodePos == BarcodePipeline.BarcodePosition.CENTER) {
            random2();
        }
        else if (barcodePos == BarcodePipeline.BarcodePosition.RIGHT) {
            random3();
        }

        drive.simpleBrake();
        sleep(500);

    }

    public void random1() {
        drive.simpleMoveToPosition(600, Constants.AUTO_LINEAR_SPEED);
        drive.tankRotate(Math.PI/(-2), Constants.AUTO_ROTATIONAL_SPEED);
        drive.simpleMoveToPosition(-50, MecDriveV2.MovementType.STRAFE, Constants.AUTO_LINEAR_SPEED);

        spikeScore();
        drive.simpleMoveToPosition(100, Constants.AUTO_LINEAR_SPEED);
        drive.simpleMoveToPosition(-600, MecDriveV2.MovementType.STRAFE, Constants.AUTO_LINEAR_SPEED);
        drive.tankRotate(Math.PI/(-2), Constants.AUTO_ROTATIONAL_SPEED);

        sleep(5000);
        drive.simpleMoveToPosition(-1000, Constants.AUTO_LINEAR_SPEED);
        drive.simpleMoveToPosition(-300, MecDriveV2.MovementType.STRAFE, Constants.AUTO_LINEAR_SPEED);
        drive.simpleMoveToPosition(-500, Constants.AUTO_LINEAR_SPEED);
        creep();
        autoScore();
        drive.simpleMoveToPosition(50, Constants.AUTO_LINEAR_SPEED);
        drive.simpleMoveToPosition(400, MecDriveV2.MovementType.STRAFE, Constants.AUTO_LINEAR_SPEED);
        drive.simpleMoveToPosition(-100, Constants.AUTO_LINEAR_SPEED);
    }

    public void random2() {
        drive.simpleMoveToPosition(1200, Constants.AUTO_LINEAR_SPEED);
        drive.simpleMoveToPosition(-100, MecDriveV2.MovementType.STRAFE, Constants.AUTO_LINEAR_SPEED);

        spikeScore();
        drive.simpleMoveToPosition(100, Constants.AUTO_LINEAR_SPEED);
        drive.tankRotate(-(3.12/2), Constants.AUTO_ROTATIONAL_SPEED);
        drive.simpleMoveToPosition(-1900, Constants.AUTO_LINEAR_SPEED);
        drive.simpleMoveToPosition(-500, MecDriveV2.MovementType.STRAFE, Constants.AUTO_LINEAR_SPEED);
        drive.simpleMoveToPosition(-100, Constants.AUTO_LINEAR_SPEED);
        creep();
        autoScore();
        drive.simpleMoveToPosition(100, Constants.AUTO_LINEAR_SPEED);
        sleep(500);
        drive.simpleMoveToPosition(700, MecDriveV2.MovementType.STRAFE, Constants.AUTO_LINEAR_SPEED);
        drive.simpleMoveToPosition(-200, Constants.AUTO_LINEAR_SPEED);

    }

    public void random3() {
        drive.simpleMoveToPosition(600, Constants.AUTO_LINEAR_SPEED);
        drive.tankRotate(Math.PI/(2), Constants.AUTO_ROTATIONAL_SPEED);

        /*
        score.setIntakePower(-0.2);
        sleep(500);
        score.setIntakePower(0);
         */
        spikeScore();
        drive.simpleMoveToPosition(150, Constants.AUTO_LINEAR_SPEED);
        drive.simpleMoveToPosition(600, MecDriveV2.MovementType.STRAFE, Constants.AUTO_LINEAR_SPEED);

        drive.simpleMoveToPosition(-1000, Constants.AUTO_LINEAR_SPEED);

        creep();
        autoScore();
        drive.simpleMoveToPosition(50, Constants.AUTO_SAFE_MO);
        drive.simpleMoveToPosition(500, MecDriveV2.MovementType.STRAFE, Constants.AUTO_LINEAR_SPEED);
        drive.simpleMoveToPosition(-100, Constants.AUTO_LINEAR_SPEED);

    }

    public void spikeScore() {
        drive.simpleMoveToPosition(-50, Constants.AUTO_SAFE_MO);
        score.setBumperPixelRelease(Constants.AUTO_SCORING_CLAMP_OPEN);
        drive.simpleMoveToPosition(100, Constants.AUTO_SLOWED_SPEED);
        score.setBumperPixelRelease(Constants.AUTO_SCORING_CLAMP_CLOSED);
        drive.simpleMoveToPosition(100, Constants.AUTO_SAFE_MO);

    }
    public void creep() {
        drive.setPowerAuto(Constants.AUTO_SLOWED_SPEED, MecDriveV2.MovementType.STRAIGHT);
        //Change me!
        sleep(600);
        drive.setPowerAuto(0, MecDriveV2.MovementType.STRAIGHT);
    }

    public void autoScore() {
        score.setLinkagePositionLogistic(0.5, 1000, 100);
        //  score.goToLiftTarget(Constants.LIFT_LOW, 0.8);
        score.setLinkagePositionLogistic(Constants.LINKAGE_UP, 1500, 100);
        sleep(300);
        score.setGrabberPosition(Constants.OPEN);
        sleep(300);
        score.setLinkagePositionLogistic(0.5, 1000, 100);
        //   score.goToLiftTarget(0, 0.3);
        score.setLinkagePositionLogistic(Constants.LINKAGE_DOWN, 1000, 100);

    }

}
