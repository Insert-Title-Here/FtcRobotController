package org.firstinspires.ftc.teamcode.MultiAuto;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Auto.ContourMultiScoreLeft;
import org.firstinspires.ftc.teamcode.Auto.DetectionAlgorithmRight;
import org.firstinspires.ftc.teamcode.Common.Constants;
import org.firstinspires.ftc.teamcode.Common.MecanumDrive;
import org.firstinspires.ftc.teamcode.Common.ScoringSystem;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.concurrent.atomic.AtomicBoolean;

//@Autonomous
public class MultiRedRightMedium extends LinearOpMode {
    MecanumDrive drive;
    ScoringSystem score;
    AtomicBoolean cont;
    Thread liftThread;
    ContourMultiScoreLeft detect;
    DetectionAlgorithmRight park;
    Constants constants;
    OpenCvWebcam webcam;

    public double position = -0.1;
    private double properCX = 89;

    @Override
    public void runOpMode() throws InterruptedException {
        detect = new ContourMultiScoreLeft(telemetry);
        park = new DetectionAlgorithmRight(telemetry);
        drive = new MecanumDrive(hardwareMap, telemetry);
        score = new ScoringSystem(hardwareMap, telemetry);
        constants = new Constants();
        cont = new AtomicBoolean();
        cont.set(false);



        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        webcam.setPipeline(detect);

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

        //TODO: Possibly change turns from encoder to IMU angles
        //TODO: Work on auto for all the side (make different methods for each side?)

        liftThread = new Thread() {
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


        // code to turn servo of cam
        score.setCamPosition(position);
        detect.park = true;
//        // ftc dashboard
//        FtcDashboard.getInstance().startCameraStream(webcam, 0);
//        telemetry.addData("Status", "Initialized");

        telemetry.update();

        //close claw
        score.setClawPosition(constants.getClawClosePos());
        waitForStart();
        detect.park = false;
        // turn servo of cam forward for poles
        score.setCamPosition(0.15);
        blueRight();
    }
    public void blueRight() throws InterruptedException {
        //lift claw a little bit
        score.goToPosition(50, 0.7);
        // go forward next to pole
        drive.goToPositionPID(drive.avgPosition(1476, 1456, 1447, 1442), "go forward next to pole");
        // turn to right 45 degrees to medium pole
        drive.turn45(Math.PI / 4);
        // go to pole a bit
        drive.goToPosition(0.3, 0.3, 0.3, 0.3, 200, "go forward some to pole");
        // camera position correction
        if (detect.getcX() < properCX - 5 || detect.getcX() > properCX + 5) {
            while (detect.getcX() < properCX - 5) {
                // strafe to the left
                drive.goToPosition(-0.2, 0.2, 0.2, -0.2);

            }
            while (detect.getcX() > properCX + 5) {
                // strafe to the right (change fr and bl)
                drive.goToPosition(0.2, -0.2, -0.2, 0.2);

            }
        }

        // scoring cone
        scoreCone(438, 416, 437, 426);
        // turn back straight
        drive.turn45(Math.PI / 4);
        //go forward to blue cone tape adjacent mat
        drive.goToPositionPID( drive.avgPosition(1028, 1056, 1041, 1026), "go forward to next mat");
        // turn to tape/cones
        drive.turn45(-Math.PI / 2);
        // find tape, get cone
        useColorSensor();
        // back up
        drive.goToPosition(-0.4, -0.4, -0.4, -0.4, -284, "back up");
        //put lift down

        // turn 90 to the left
        drive.turn45(Math.PI / 2);
        // strafe left
        drive.goToPosition(0.4, -0.4, -0.4, 0.4, drive.avgPosition(1727, -1651, -1650, 891), "strafe right");
        // camera position correction
        if (detect.getcX() < properCX - 5 || detect.getcX() > properCX + 5) {
            while (detect.getcX() < properCX - 5) {
                // strafe to the left
                drive.goToPosition(-0.2, 0.2, 0.2, -0.2);

            }
            while (detect.getcX() > properCX + 5) {
                // strafe to the right (change fr and bl)
                drive.goToPosition(0.2, -0.2, -0.2, 0.2);

            }
        }
        // scoring cone
        scoreCone(184, 165, 163, 147);
/*
        //moves robot to correct parking position
        if (detect.getPosition() == DetectionAlgorithmTest.ParkingPosition.LEFT) {
            // move to left park (strafe right)
            drive.goToPosition(-0.5, 0.5, 0.5, -0.5, drive.avgPosition(-5007,2941,3226,-3036), "strafe left (more left)");
            drive.goToPosition(0.3, 0.3, 0.3, 0.3, drive.avgPosition(600,600,600,650), "strafe right");


        } else if (detect.getPosition() == DetectionAlgorithmTest.ParkingPosition.CENTER) {
            // move to center park (strafe left)
            drive.goToPosition(-0.5, 0.5, 0.5, -0.5, drive.avgPosition(-1759,1748,1937,-1784), "strafe left (center)");
            drive.goToPosition(0.3, 0.3, 0.3, 0.3, drive.avgPosition(400,400,400,400), "strafe right");

        } else {
            // move to right park (strafe more left)
            drive.goToPosition(-0.3, 0.3, 0.3, -0.3, drive.avgPosition(-560,565,642,-585), "strafe left");
            drive.goToPosition(0.3, 0.3, 0.3, 0.3, drive.avgPosition(400,400,400,400), "strafe right");


        }
*/

        score.setClawPosition(constants.getClawOpenPos());



    }

    public void scoreCone(int fl, int fr, int bl, int br) {

        // move arm max
        score.goToPosition(2340, 0.85);
        //begin thread for maintaining height of slides
        cont.set(true);
        //move forward closer to pole
        if (detect.getDistance() >= 6 || detect.getDistance() <= 4) {
            while (detect.getDistance() >= 6) {
                drive.goToPosition(0.2, 0.2, 0.2, 0.2);
            }
            while (detect.getDistance() < 5) {
                drive.goToPosition(-0.2, -0.2, -0.2, -0.2);
            }
        }
        sleep(500);

        //lower cone onto pole
        score.goToPosition(score.getEncoderPosition()-300, 0.4);
        score.setClawPosition(constants.getClawOpenPos());
        sleep(300);

        //move back from pole
        drive.goToPosition(-0.3, -0.3, -0.3, -0.3, drive.avgPosition(fl, fr, bl, br), "move back from pole");
        cont.set(false);
        //moves slides down
        score.goToPosition(0, 0.3);
        sleep(300);
    }

    public void useColorSensor() {
        //drive.findTape();
        score.goToPosition(174, 0.7);
        score.grabConeAuto();


    }
    public void useCam() {

    }
}
