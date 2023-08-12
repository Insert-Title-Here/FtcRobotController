package org.firstinspires.ftc.teamcode.Competition.DebuggingOrTuning.Vision.InUse;

//import com.acmerobotics.dashboard.FtcDashboard;
////import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Competition.State.Subsystems.Current.KevinGodPipelineAprilTag;
import org.firstinspires.ftc.teamcode.Competition.Interleagues.Common.Constants;
import org.firstinspires.ftc.teamcode.Competition.State.Subsystems.Current.MecDriveV2;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;


@TeleOp
public class XContourDetectorTest extends LinearOpMode {
    Servo servo;
    OpenCvWebcam camera;
    KevinGodPipelineAprilTag pipeline;
    MecDriveV2 drive;


    @Override
    public void runOpMode() throws InterruptedException {
        servo = hardwareMap.get(Servo.class, "camera");
        drive = new MecDriveV2(hardwareMap, false, telemetry);

        WebcamName webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);
        pipeline = new KevinGodPipelineAprilTag(telemetry, drive, KevinGodPipelineAprilTag.AutoSide.RED_RIGHT, false);
        camera.setPipeline(pipeline);

        servo.setPosition(Constants.poleV2);


        //FtcDashboard.getInstance().startCameraStream(camera, 0);


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
        pipeline.changeMode(KevinGodPipelineAprilTag.Mode.POLE);

        while(opModeInInit()){

            //double yPos = getYCapPosition();
            //setYCapPosition(yPos - map(gamepad1.right_stick_y, -1, 1, -0.0010, 0.0010));
            //telemetry.addData("Position", getYCapPosition());

            if(gamepad1.a){
                servo.setPosition(Constants.poleV2);
                pipeline.changeMode(KevinGodPipelineAprilTag.Mode.POLE);
            }

            if(gamepad1.b){
                servo.setPosition(Constants.coneV2);
                pipeline.changeMode(KevinGodPipelineAprilTag.Mode.BLUECONE);
            }

            if(gamepad1.x){
                servo.setPosition(Constants.coneV2);
                pipeline.changeMode(KevinGodPipelineAprilTag.Mode.REDCONE);
            }


            telemetry.addData("A", "Press A for pole x contour");
            telemetry.addData("B", "Press B for blue cone x contour");
            telemetry.addData("X", "Press X for red cone x contour");
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
