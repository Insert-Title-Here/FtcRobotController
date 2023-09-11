package org.firstinspires.ftc.teamcode.Competition.DebuggingOrTuning.Vision.InUse.NormalizationTests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Competition.State.Subsystems.Current.KevinGodPipelineAprilTag;
import org.firstinspires.ftc.teamcode.Competition.Interleagues.Common.Constants;
import org.firstinspires.ftc.teamcode.Competition.State.Subsystems.Current.MecDriveV2;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

@Config
@TeleOp
public class RedConeNormalizationTest extends LinearOpMode {


    public static double servoHeight = Constants.coneV2;

    boolean aFlag = true;

    OpenCvWebcam camera;
    KevinGodPipelineAprilTag pipeline;
    Servo cameraServo;
    MecDriveV2 drive;
    ColorRangeSensor color;

    @Override
    public void runOpMode() throws InterruptedException {
        WebcamName webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);
        cameraServo = hardwareMap.get(Servo.class, "camera");
        //color = hardwareMap.get(ColorRangeSensor.class, "color");
        drive = new MecDriveV2(hardwareMap, false, telemetry, true);
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

        pipeline.changeMode(KevinGodPipelineAprilTag.Mode.REDCONE);


        while(opModeInInit()){
            if(gamepad1.a) {
                pipeline.normalizeStrafe(0.4, 150, 2);
            }

            if(gamepad1.b){
                pipeline.normalize(0.25, 150, 2);
            }
            cameraServo.setPosition(servoHeight);

            telemetry.addData("A", "Press A for strafe normalization");
            telemetry.addData("B", "Press B for rotate normalization");
            telemetry.addData("pole x pos", pipeline.getXContour());
            telemetry.update();

        }


        waitForStart();
        //pipeline.normalize(-0.2, 160, 5);


        //SignalPipeline.ParkPos position = pipeline.getPosition();



        camera.closeCameraDevice();
    }
}

