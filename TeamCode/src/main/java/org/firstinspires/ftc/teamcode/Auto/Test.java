package org.firstinspires.ftc.teamcode.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Common.MecanumDrive;
import org.firstinspires.ftc.teamcode.Common.ScoringSystem;

import java.util.concurrent.atomic.AtomicBoolean;
/*
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

 */

//@Autonomous
public class Test extends LinearOpMode {
    MecanumDrive drive;
    ScoringSystem score;
    //OpenCvWebcam webcam;
    AtomicBoolean cont;
    Thread liftThread;
    int parkLocation;
    @Override
    public void runOpMode() throws InterruptedException {
        drive = new MecanumDrive(hardwareMap, telemetry);
        score = new ScoringSystem(hardwareMap, telemetry);
        cont = new AtomicBoolean();
        cont.set(false);
        /*
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "webcam"), cameraMonitorViewId);
        telemetry.addData("Status", "Initialized");
        telemetry.update();
         */

        // Camera checks sleeve...stores parking location??

        //TODO: Possibly change turns from encoder to IMU angles
        //TODO: Work on auto for all the side (make different methods for each side?)

        liftThread = new Thread(){
            @Override
            public void run(){
                while(opModeIsActive()){
                    if((score.getEncoderPosition() > 1200 && cont.get())){
                        score.setPower(0.1);
                    }


                    telemetry.addData("liftPow", score.getPower());
                    telemetry.addData("liftPos", score.getEncoderPosition());
                    telemetry.update();
                }

            }
        };
        //TRY SETTING THE COMMANDS INSIDE THE THREAD AND SEE IF IT WORKS THAT WAY


        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();

        liftThread.start();
        test();




    }

    public void test() {
         //drive.goToPositionTest(1224, 1228, 1185, 1176, 0.3, "forward");
         drive.goToPosition(1840, -1720, -1786, 1968, 3, "right");
    }

}

