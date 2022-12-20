package org.firstinspires.ftc.teamcode.League1.Autonomous.Vision.Demo;

////import com.acmerobotics.dashboard.FtcDashboard;
////import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.League1.Autonomous.Vision.KevinGodPipeline;
import org.firstinspires.ftc.teamcode.League1.Autonomous.Vision.SignalPipeline;
import org.firstinspires.ftc.teamcode.League1.Common.Constants;
import org.firstinspires.ftc.teamcode.League1.Subsystems.MecDrive;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

////@Config
@Autonomous
public class VisionDemo extends LinearOpMode {


    public static double servoHeight = Constants.pole;

    OpenCvWebcam camera;
    NormalizationDemoPipeline pipeline;
    Servo cameraServo;

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize camera and servo
        WebcamName webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);
        cameraServo = hardwareMap.get(Servo.class, "camera");
        pipeline = new NormalizationDemoPipeline(telemetry, cameraServo);

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

        while(opModeInInit()){
            if(gamepad1.a) {
                while(gamepad1.a) {

                }
                pipeline.normalizeToPole(70, 5);
                sleep(2000);
            } else {
                cameraServo.setPosition(servoHeight);
            }
        }


        waitForStart();


        camera.closeCameraDevice();
    }
}

