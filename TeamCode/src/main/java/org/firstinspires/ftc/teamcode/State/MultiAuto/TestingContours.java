package org.firstinspires.ftc.teamcode.State.MultiAuto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.State.Auto.NormalizationTesting;
import org.firstinspires.ftc.teamcode.State.Common.Constants;
import org.firstinspires.ftc.teamcode.State.Common.MecanumDrive;
import org.firstinspires.ftc.teamcode.State.Common.ScoringSystem;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.concurrent.atomic.AtomicBoolean;

@Autonomous @Config
public class TestingContours extends LinearOpMode {
    MecanumDrive drive;
    ScoringSystem score;
    AtomicBoolean cont;
    Thread liftThread;
    NormalizationTesting park;
    Constants constants;
    OpenCvWebcam webcam;


    private double properCX = 89;
    public static double test;

    @Override
    public void runOpMode() throws InterruptedException {

        park = new NormalizationTesting(telemetry);
        drive = new MecanumDrive(hardwareMap, telemetry);
        //sets Auto driving to break instead of float
        drive.mecanumDriveAuto(hardwareMap, telemetry);
        score = new ScoringSystem(hardwareMap, telemetry);
        constants = new Constants();
        cont = new AtomicBoolean();
        cont.set(false);
        test = constants.getStrafeLowCamPos();




        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        webcam.setPipeline(park);

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

        // Camera checks sleeve...stores parking location??

        //TODO: Possibly change turn45s from encoder to IMU angles
        //TODO: Work on auto for all the side (make different methods for each side?)

        liftThread = new Thread() {
            @Override
            public void run(){
                while(opModeIsActive()){
                    if((score.getEncoderPosition() > 200 && cont.get())){
                        score.setPower(constants.getSteadyPow());
                    }


                    telemetry.addData("liftPow", score.getLeftPower());
                    telemetry.addData("liftPos", score.getEncoderPosition());
                    telemetry.update();
                }

            }
        };



        // code to turn45 servo of cam
        score.setCamPosition(constants.getSleeveCamPos());

        // ftc dashboard
        FtcDashboard.getInstance().startCameraStream(webcam, 0);
        telemetry.addData("Status", "Initialized");

        telemetry.update();

        //close claw
        score.setClawPosition(constants.getClawClosePos());


        waitForStart();
        liftThread.start();
        webcam.stopStreaming();

        blueLeft();
    }
    public void blueLeft() throws InterruptedException {
        //lift claw a little bit
        score.goToPosition(50, 0.8);
        sleep(100);
        // go forward next to pole
        drive.goToPosition(0.9, 0.8, 0.9, 0.8, 1350, "go forward next to pole");
        // turn to right 45 degrees to high pole
        drive.absTurnDriftPID(-Math.PI / 4);
        //moves robot to correct parking position
//        if (park.getPosition() == DetectionAlgorithmLeft.ParkingPosition.LEFT) {
//            // move to left park (strafe right)
//            drive.goToPosition(-0.3, -0.3, -0.3, -0.3, 1372, "move backwards");
//            drive.goToPosition(-0.3, 0.3, 0.3, -0.3 , 200, "strafe right");
//
//
//
//        } else if (park.getPosition() == DetectionAlgorithmLeft.ParkingPosition.CENTER) {
//            // move to center park (don't move at all)
//            drive.goToPosition(-0.3, 0.3, 0.3, -0.3 , 200, "strafe right");
//
//        } else {
//            // move to right park (strafe more left)
//            drive.goToPosition(0.3, 0.3, 0.3, 0.3 , 1000, "move forward");
//            drive.goToPosition(-0.3, 0.3, 0.3, -0.3 , 200, "strafe right");
//
//
//        }


        score.setClawPosition(constants.getClawOpenPos());



    }


    public void scoreCone(int fl, int fr, int bl, int br) {
        cont.set(true);
        // move arm med
        score.goToPosition(constants.getHeightMed(), 0.8);
        //begin thread for maintaining height of slides


        //move forward to pole
        drive.goToPosition(0.3, 0.3, 0.3, 0.3, drive.avgPosition(fl, fr, bl, br), "move to pole");
        sleep(200);

        //lower cone onto pole
        score.goToPosition(score.getEncoderPosition()-300, 0.4);
        score.setClawPosition(constants.getClawOpenPos());
        sleep(300);

        //move back from pole
        drive.goToPosition(-0.3, -0.3, -0.3, -0.3, drive.avgPosition(fl+200, fr, bl, br), "move back from pole");
        cont.set(false);
        //moves slides down
        score.goToPosition(0, 0.3);
        sleep(300);
    }

    public void useColorSensor() {
        drive.findTape("blue", true);
        score.goToPosition(174, 0.7);
        score.grabConeAuto();


    }
    public void useCam() {

    }
}
