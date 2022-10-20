package org.firstinspires.ftc.teamcode.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Common.MecanumDrive;
import org.firstinspires.ftc.teamcode.Common.ScoringSystem;

import java.util.concurrent.atomic.AtomicBoolean;
//TODO:Needs testing
@Autonomous
public class NewGoToPosTest extends LinearOpMode {
    MecanumDrive drive;
    ScoringSystem score;
    //OpenCvWebcam webcam;
    AtomicBoolean cont;
    Thread liftThread;
    int parkLocation;
    @Override
    public void runOpMode() throws InterruptedException {
        drive = new MecanumDrive(hardwareMap, telemetry);
        score = new ScoringSystem(hardwareMap);
        cont = new AtomicBoolean();
        cont.set(false);
        /*
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "webcam"), cameraMonitorViewId);
        telemetry.addData("Status", "Initialized");
        telemetry.update();
         */

        // Camera checks sleeve...stores parking location??

        liftThread = new Thread(){
            @Override
            public void run(){
                while(opModeIsActive()){
                    if((score.getEncoderPosition() > 1200 && cont.get())){
                        score.setPower(0.1);
                    }



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
    public void test(){
        drive.goToPositionTest(500,500,500,500,0.3,"forward test");
    }

}
