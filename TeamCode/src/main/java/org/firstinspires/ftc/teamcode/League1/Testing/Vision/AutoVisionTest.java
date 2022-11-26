package org.firstinspires.ftc.teamcode.League1.Testing.Vision;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.League1.Autonomous.Vision.ContourPipeline;
import org.firstinspires.ftc.teamcode.League1.Autonomous.Vision.KevinGodPipeline;
import org.firstinspires.ftc.teamcode.League1.Autonomous.Vision.KevinGodPipelineV2;
import org.firstinspires.ftc.teamcode.League1.Common.Constants;
import org.firstinspires.ftc.teamcode.League1.Subsystems.MecDrive;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;


@TeleOp
public class AutoVisionTest extends LinearOpMode {
    Servo servo;
    OpenCvWebcam camera;
    KevinGodPipelineV2 pipeline;
    MecDrive drive;


    @Override
    public void runOpMode() throws InterruptedException {
        servo = hardwareMap.get(Servo.class, "camera");
        drive = new MecDrive(hardwareMap, false, telemetry);

        WebcamName webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);
        pipeline = new KevinGodPipelineV2(telemetry, drive, KevinGodPipelineV2.AutoSide.RED_RIGHT);

        camera.setPipeline(pipeline);

        servo.setPosition(Constants.cone);

        FtcDashboard.getInstance().startCameraStream(camera, 0);


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
        pipeline.changeMode(KevinGodPipelineV2.Mode.REDCONE);

        while(opModeInInit()){

            double yPos = getYCapPosition();
            setYCapPosition(yPos - map(gamepad1.right_stick_y, -1, 1, -0.0010, 0.0010));
            telemetry.addData("Position", getYCapPosition());
            telemetry.addData("XPos", pipeline.getXContour());
            telemetry.update();

        }

        waitForStart();
    }


    public void setYCapPosition(double pos){
        servo.setPosition(pos);
    }

    public double map(double val, double in_min, double in_max, double out_min, double out_max) {
        return (val - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
    }


    public double getYCapPosition() {
        return servo.getPosition();
    }

}
