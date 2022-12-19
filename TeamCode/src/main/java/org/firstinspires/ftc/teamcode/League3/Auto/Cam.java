package org.firstinspires.ftc.teamcode.League3.Auto;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import org.firstinspires.ftc.teamcode.League3.Auto.DetectionAlgorithm.ParkingPosition;

//@Autonomous
public class Cam extends LinearOpMode {
    DetectionAlgorithm detect;
    String position; //temp
    OpenCvWebcam webcam;
    int parkLocation;


    @Override
    public void runOpMode() throws InterruptedException {
        detect = new DetectionAlgorithm(telemetry);
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        webcam.setPipeline(detect);

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

        telemetry.addData("position", position);
        telemetry.addData("Status", "Initialized");

        telemetry.update();


        waitForStart();
        webcam.stopStreaming();

        if (detect.getPosition() == ParkingPosition.LEFT) {
            // move to left
        } else if (detect.getPosition() == ParkingPosition.CENTER) {
            // move to center
        } else {
            // move to right
        }
    }
}
