package org.firstinspires.ftc.teamcode.Testing.OpenCV;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.List;
//@Disabled
@TeleOp
public class AprilTagTest extends LinearOpMode {

    private AprilTagProcessor aprilTag;
    VisionPortal visionPortal;
    OpenCvWebcam camera;
    BarcodePipeline pipeline;
    BarcodePipeline.BarcodePosition barcodePos;
    public void runOpMode() {

        WebcamName webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);
        pipeline = new BarcodePipeline();

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

        waitForStart();

        barcodePos = pipeline.getPos();
        camera.closeCameraDevice();

        initAprilTags();

        while(opModeIsActive()) {

            List<AprilTagDetection> detections = aprilTag.getDetections();

            for (AprilTagDetection tag : detections) {
                if (tag.metadata != null) {
                    telemetry.addData("Tag ID", tag.id);
                    telemetry.addData("X pos", tag.ftcPose.x);
                    telemetry.addData("Y pos", tag.ftcPose.y);
                    telemetry.addData("Z pos", tag.ftcPose.z);
                    // thing
                }
            }

            telemetry.update();
        }

    }

    public void initAprilTags() {
        aprilTag = new AprilTagProcessor.Builder().build();

        VisionPortal.Builder builder = new VisionPortal.Builder();
        builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        builder.addProcessor(aprilTag);
        visionPortal = builder.build();

    }
}
