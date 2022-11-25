package org.firstinspires.ftc.teamcode.Auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Common.MecanumDrive;
import org.firstinspires.ftc.teamcode.Common.ScoringSystem;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.concurrent.atomic.AtomicBoolean;

@Autonomous
public class CamwithContours extends LinearOpMode {
    MecanumDrive drive;
    ScoringSystem score;
    AtomicBoolean cont;
    Thread liftThread;
    ContourMultiScore detect;
    DetectionAlgorithmTest park;
    OpenCvWebcam webcam;

    public double position = -0.1;

    @Override
    public void runOpMode() throws InterruptedException {
        detect = new ContourMultiScore(telemetry);
        park = new DetectionAlgorithmTest(telemetry);
        drive = new MecanumDrive(hardwareMap, telemetry);
        score = new ScoringSystem(hardwareMap, telemetry);
        cont = new AtomicBoolean();
        cont.set(false);



        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        webcam.setPipeline(detect);

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {

            @Override
            public void onOpened() {
                webcam.startStreaming(320, 176, OpenCvCameraRotation.UPSIDE_DOWN);
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
        // ftc dashboard
        FtcDashboard.getInstance().startCameraStream(webcam, 0);
        telemetry.addData("Status", "Initialized");

        telemetry.update();


        waitForStart();
        detect.park = false;
        // turn servo of cam forward for poles
        score.setCamPosition(0);
        blueLeft();
    }
    public void blueLeft() throws InterruptedException {
        // go forward next to pole
        drive.goToPositionPID( drive.avgPosition(1376, 1356, 1347, 1342), "go forward next to pole");
        // turn to left 45 degrees to medium pole
        drive.turn(-Math.PI / 4, 0);
        // camera position correction
        if (detect.getcX() < 160) { //TODO: figure out value for 160
            while (detect.getcX() < 160) {
                // strafe to the right
                drive.goToPosition(0.4, -0.4, -0.4, 0.4);

            }
        } else if (detect.getcX() > 53) {
            while (detect.getcX() > 160) {
                // strafe to the left (change fr and bl)
                drive.goToPosition(-0.4, 0.4, 0.4, -0.4);

            }
        } else { // cX must equal to 160
            // move forward until close enough

        }
        // scoring cone
        scoreCone(438, 416, 437, 426);
        // turn back straight
        drive.turn(-Math.PI / 4, 0);
        //go forward to blue cone tape adjacent mat
        drive.goToPositionPID( drive.avgPosition(1028, 1056, 1041, 1026), "go forward to next mat");
        // turn to tape/cones
        drive.turn(Math.PI / 2, 0);
        // find tape, get cone
        useColorSensor();
        // back up
        drive.goToPosition(-0.4, -0.4, -0.4, -0.4, -284, "back up");
        //put lift down

        // turn 90 to the right
        drive.turn(-Math.PI / 2, 0);
        // strafe right
        drive.goToPosition(-0.4, 0.4, 0.4, -0.4, drive.avgPosition(-1727, 1651, 1650, -1601), "strafe right");
        // camera position correction
        if (detect.getcX() < 160) { //TODO: figure out value for 160
            while (detect.getcX() < 160) {
                // strafe to the right
                drive.goToPosition(0.4, -0.4, -0.4, 0.4);

            }
        } else if (detect.getcX() > 160) {
            while (detect.getcX() > 160) {
                // strafe to the left (change fr and bl)
                drive.goToPosition(-0.4, 0.4, 0.4, -0.4);

            }
        } else { // cX must equal to 160
            // move forward until close enough
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

        score.setClawPosition(0);



    }

    public void scoreCone(int fl, int fr, int bl, int br) {

        // move arm max
        score.goToPosition(2340, 0.85);
        //begin thread for maintaining height of slides
        cont.set(true);
        //move forward closer to pole
        drive.goToPosition(0.3, 0.3, 0.3, 0.3, drive.avgPosition(fl, fr, bl, br), "move to pole");
        sleep(1000);

        //lower cone onto pole
        score.goToPosition(score.getEncoderPosition()-300, 0.4);
        score.setClawPosition(0);
        sleep(300);

        //move back from pole
        drive.goToPosition(-0.3, -0.3, -0.3, -0.3, drive.avgPosition(fl, fr, bl, br), "move back from pole");
        cont.set(false);
        //moves slides down
        score.goToPosition(0, 0.3);
        sleep(300);
    }

    public void useColorSensor() {
        drive.findTape();
        score.goToPosition(174, 0.7);
        score.grabConeAuto();


    }
    public void useCam() {

    }
}
