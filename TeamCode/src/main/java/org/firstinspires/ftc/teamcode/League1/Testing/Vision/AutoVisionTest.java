package org.firstinspires.ftc.teamcode.League1.Testing.Vision;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.League1.Autonomous.Vision.ContourPipeline;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

@Disabled
@TeleOp
public class AutoVisionTest extends LinearOpMode {
    Servo servo;
    OpenCvWebcam camera;
    ContourPipeline pipeline;


    @Override
    public void runOpMode() throws InterruptedException {
        servo = hardwareMap.get(Servo.class, "camera");

        WebcamName webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);
        pipeline = new ContourPipeline(telemetry);

        camera.setPipeline(pipeline);

        servo.setPosition(.73);


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

        while(opModeInInit()){

            double yPos = getYCapPosition();
            setYCapPosition(yPos - map(gamepad1.right_stick_y, -1, 1, -0.0010, 0.0010));
            telemetry.addData("camera position", getYCapPosition());
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
