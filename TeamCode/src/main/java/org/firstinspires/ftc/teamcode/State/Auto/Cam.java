package org.firstinspires.ftc.teamcode.State.Auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.State.Common.Constants;
import org.firstinspires.ftc.teamcode.State.Common.ScoringSystem;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

@Autonomous @Config
public class Cam extends LinearOpMode {
    DetectionAlgorithmLeft detect;
    String position; //temp
    OpenCvWebcam webcam;
    ScoringSystem score;
    Constants constants = new Constants();
    int parkLocation;
    public static double pos = 0;


    @Override
    public void runOpMode() throws InterruptedException {
        detect = new DetectionAlgorithmLeft(telemetry);
        score = new ScoringSystem(hardwareMap, telemetry);
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        webcam.setPipeline(detect);
        pos = constants.getSleeveCamPos();


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


        FtcDashboard.getInstance().startCameraStream(webcam, 0);

        telemetry.addData("position", position);
        telemetry.addData("Status", "Initialized");

        telemetry.update();

        while (opModeInInit()) {
            score.setCamPosition(pos);
        }
        waitForStart();
        webcam.stopStreaming();


    }
}
