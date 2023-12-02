//blue path 2 plows over team game element
package org.firstinspires.ftc.teamcode.Autonomous.DoneAutos;

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
import org.firstinspires.ftc.teamcode.Testing.OpenCV.BarcodePipelineBlue;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.List;

//@Disabled
@Autonomous
public class BlueFrontAutoTest extends LinearOpMode {
    AprilTagProcessor aprilTag;

    MecDriveV2 drive;
    ScoringSystem score;
    ElapsedTime time;
    VisionPortal visionPortal;
    OpenCvWebcam camera;
    BarcodePipelineBlue pipeline;
    BarcodePipelineBlue.BarcodePosition barcodePos;
    public void runOpMode() {

        time = new ElapsedTime();
        drive = new MecDriveV2(hardwareMap, false, telemetry, time);
        score = new ScoringSystem(hardwareMap, telemetry, time);
        //  score.setGrabberPosition(Constants.OPEN);
        score.setGrabberPosition(Constants.GRABBING);
        score.setBumperPixelRelease(Constants.AUTO_SCORING_CLAMP_CLOSED);
        score.setIntakeLiftPos(Constants.INTAKE_LINKAGE_DOWN-0.1);
        score.setIntakeLiftPos(0.1);

        WebcamName webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);
        pipeline = new BarcodePipelineBlue(telemetry);

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
        camera.closeCameraDevice();
        initAprilTags();

        //   score.setBumperPixelRelease(Constants.AUTO_SCORING_CLAMP_CLOSED);
        sleep(1000);

        if (barcodePos == BarcodePipelineBlue.BarcodePosition.LEFT) {
            random1();
        }
        else if (barcodePos == BarcodePipelineBlue.BarcodePosition.CENTER) {
            random2();
        }
        else if (barcodePos == BarcodePipelineBlue.BarcodePosition.RIGHT) {
            random3();
        }

        score.setBumperPixelRelease(Constants.AUTO_SCORING_CLAMP_CLOSED);
        drive.simpleBrake();
        sleep(500);

    }

    public void random1() {
        drive.simpleMoveToPosition(-100, MecDriveV2.MovementType.STRAFE,Constants.AUTO_LINEAR_SPEED);
        drive.simpleMoveToPosition(400, Constants.AUTO_SLOWED_SPEED);
        drive.tankRotate(Math.PI/(4), Constants.AUTO_ROTATIONAL_SPEED);
        sleep(500);
        drive.simpleMoveToPosition(350, Constants.AUTO_SLOWED_SPEED);
        drive.simpleMoveToPosition(-120, Constants.AUTO_SLOWED_SPEED);
        score.setBumperPixelRelease(Constants.AUTO_SCORING_CLAMP_OPEN);
        drive.simpleMoveToPosition(-150, Constants.AUTO_SLOWED_SPEED);
        sleep(500);
        score.setBumperPixelRelease(Constants.AUTO_SCORING_CLAMP_CLOSED);
        drive.simpleMoveToPosition(-20, Constants.AUTO_SLOWED_SPEED);

        /*creep();
        autoScore();
        drive.simpleMoveToPosition(50, Constants.AUTO_SAFE_MO);
        drive.simpleMoveToPosition(-500, MecDriveV2.MovementType.STRAFE, Constants.AUTO_LINEAR_SPEED);
        drive.simpleMoveToPosition(-100, Constants.AUTO_LINEAR_SPEED);*/

    }


    public void random2() {
        drive.simpleMoveToPosition(-100, MecDriveV2.MovementType.STRAFE,Constants.AUTO_LINEAR_SPEED);
        sleep(500);
        drive.simpleMoveToPosition(850, Constants.AUTO_SLOWED_SPEED);
        drive.simpleMoveToPosition(-120, Constants.AUTO_SLOWED_SPEED);
        score.setBumperPixelRelease(Constants.AUTO_SCORING_CLAMP_OPEN);
        drive.simpleMoveToPosition(-100, Constants.AUTO_SLOWED_SPEED);
        sleep(500);
        score.setBumperPixelRelease(Constants.AUTO_SCORING_CLAMP_CLOSED);

        /*spikeScore();
        drive.tankRotate((3.12/2), Constants.AUTO_ROTATIONAL_SPEED);
        sleep(5000);
        drive.simpleMoveToPosition(-1900, Constants.AUTO_LINEAR_SPEED);
        drive.simpleMoveToPosition(600, MecDriveV2.MovementType.STRAFE, Constants.AUTO_LINEAR_SPEED);
        creep();
        autoScore();
        drive.simpleMoveToPosition(200, Constants.AUTO_LINEAR_SPEED);
        drive.simpleMoveToPosition(-700, MecDriveV2.MovementType.STRAFE, Constants.AUTO_LINEAR_SPEED);
        drive.simpleMoveToPosition(-300, Constants.AUTO_LINEAR_SPEED);*/

    }

    public void random3() {
        drive.simpleMoveToPosition(-100, MecDriveV2.MovementType.STRAFE,Constants.AUTO_LINEAR_SPEED);
        drive.simpleMoveToPosition(600, Constants.AUTO_SLOWED_SPEED);
        drive.tankRotate(Math.PI/(-2.1), Constants.AUTO_ROTATIONAL_SPEED);
        sleep(500);
        drive.simpleMoveToPosition(-75, Constants.AUTO_SLOWED_SPEED);
        score.setBumperPixelRelease(Constants.AUTO_SCORING_CLAMP_OPEN);
        sleep(500);
        score.setBumperPixelRelease(Constants.AUTO_SCORING_CLAMP_CLOSED);

        /*sleep(5000);
        drive.simpleMoveToPosition(-1000, Constants.AUTO_LINEAR_SPEED);
        drive.simpleMoveToPosition(300, MecDriveV2.MovementType.STRAFE, Constants.AUTO_LINEAR_SPEED);
        normalizeStrafe(0, 0.3, 3);
        drive.simpleMoveToPosition(-500, Constants.AUTO_LINEAR_SPEED);
        creep();
        autoScore();
        drive.simpleMoveToPosition(50, Constants.AUTO_LINEAR_SPEED);
        drive.simpleMoveToPosition(-400, MecDriveV2.MovementType.STRAFE, Constants.AUTO_LINEAR_SPEED);
        drive.simpleMoveToPosition(-100, Constants.AUTO_LINEAR_SPEED);*/
    }

    public void spikeScore() {
        drive.simpleMoveToPosition(-100, Constants.AUTO_SAFE_MO);
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
    public void initAprilTags() {
        aprilTag = new AprilTagProcessor.Builder().build();

        VisionPortal.Builder builder = new VisionPortal.Builder();
        builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        builder.addProcessor(aprilTag);
        visionPortal = builder.build();

    }

    public void normalizeStrafe(double target, double power, int tagID) {
        while(Math.abs(getXPose(tagID) - target) > 0.2 && getXPose(tagID) > Integer.MIN_VALUE) {
            drive.setPowerAuto(power * Math.signum(getXPose(tagID) - target), MecDriveV2.MovementType.STRAFE);
            telemetry.addData("x", getXPose(tagID));
        }
        drive.simpleBrake();
    }

    public void normalizeStraight(double target, double power, int tagID) {
        while(Math.abs(getYPose(tagID) - target) > 0.2 && getYPose(tagID) > Integer.MIN_VALUE) {
            drive.setPowerAuto(-power * Math.signum(getYPose(tagID) - target), MecDriveV2.MovementType.STRAIGHT);
            telemetry.addData("x", getXPose(tagID));
        }
        drive.simpleBrake();
    }

    public double getXPose(int tagID) {
        List<AprilTagDetection> detections = aprilTag.getDetections();

        for (AprilTagDetection tag : detections) {
            if (tag.metadata != null && tag.id == tagID) {
                return tag.ftcPose.x;
            }
        }

        return Integer.MIN_VALUE;
    }

    public double getYPose(int tagID) {
        List<AprilTagDetection> detections = aprilTag.getDetections();

        for (AprilTagDetection tag : detections) {
            if (tag.metadata != null && tag.id == tagID) {
                return tag.ftcPose.y;
            }
        }

        return Integer.MIN_VALUE;
    }

}