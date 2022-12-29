package org.firstinspires.ftc.teamcode.League1.Testing.Vision;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.AprilTagsTesting.KevinGodPipelineAprilTag;
import org.firstinspires.ftc.teamcode.League1.Subsystems.MecDrive;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

@Config
@Autonomous
public class PoleNormalizationTest extends LinearOpMode {


    public static double servoHeight = 0.35;

    boolean aFlag = true;

    OpenCvWebcam camera;
    KevinGodPipelineAprilTag pipeline;
    Servo cameraServo;
    MecDrive drive;
    ColorRangeSensor color;

    @Override
    public void runOpMode() throws InterruptedException {
        WebcamName webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);
        cameraServo = hardwareMap.get(Servo.class, "camera");
        color = hardwareMap.get(ColorRangeSensor.class, "color");
        drive = new MecDrive(hardwareMap, false, telemetry, color);
        pipeline = new KevinGodPipelineAprilTag(telemetry, drive, KevinGodPipelineAprilTag.AutoSide.RED_RIGHT, false);

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

        /*
        while (opModeInInit()) {
            if (gamepad1.a && aFlag) {
                aFlag = false;
                pipeline.switchMat();
            }
            if (!gamepad1.a) {
                aFlag = true;
            }
        }

         */

        FtcDashboard.getInstance().startCameraStream(camera, 0);

        pipeline.changeMode(KevinGodPipelineAprilTag.Mode.POLE);


        while(opModeInInit()){
            if(gamepad1.a) {
                pipeline.normalize(0.25, 160, 2);
            }
            cameraServo.setPosition(servoHeight);

            telemetry.addData("pole x pos", pipeline.getXContour());
            telemetry.update();

        }


        waitForStart();
        //pipeline.normalize(-0.2, 160, 5);


        //SignalPipeline.ParkPos position = pipeline.getPosition();



        camera.closeCameraDevice();
    }
}

