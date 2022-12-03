package org.firstinspires.ftc.teamcode.MultiAuto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Auto.DetectionAlgorithmTest;
import org.firstinspires.ftc.teamcode.Common.Constants;
import org.firstinspires.ftc.teamcode.Common.MecanumDrive;
import org.firstinspires.ftc.teamcode.Common.ScoringSystem;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.concurrent.atomic.AtomicBoolean;

@Autonomous
public class OneBlueLeftMedium extends LinearOpMode {
    MecanumDrive drive;
    ScoringSystem score;
    AtomicBoolean cont;
    Thread liftThread;
    DetectionAlgorithmTest park;
    Constants constants;
    OpenCvWebcam webcam;

    public double position = -0.1;
    private double properCX = 89;

    @Override
    public void runOpMode() throws InterruptedException {

        park = new DetectionAlgorithmTest(telemetry);
        drive = new MecanumDrive(hardwareMap, telemetry);
        score = new ScoringSystem(hardwareMap, telemetry);
        constants = new Constants();
        cont = new AtomicBoolean();
        cont.set(false);



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

        // ftc dashboard
        FtcDashboard.getInstance().startCameraStream(webcam, 0);
        telemetry.addData("Status", "Initialized");

        telemetry.update();

        //close claw
        score.setClawPosition(constants.getClawClosePos());
        waitForStart();
        webcam.stopStreaming();

        blueLeft();
    }
    public void blueLeft() throws InterruptedException {
        //lift claw a little bit
        score.goToPosition(50, 0.7);
        // go forward next to pole
        drive.goToPosition(0.3, 0.3, 0.3, 0.3, drive.avgPosition(1468, 1380, 1380, 1359), "go forward");
        // turn to left 45 degrees to medium pole
        drive.turn(-Math.PI / 4);
        // scoring cone
        scoreCone(291, 442, 212, 402);
        // turn to straight
        drive.turn(-Math.PI / 4);

        //moves robot to correct parking position
        if (park.getPosition() == DetectionAlgorithmTest.ParkingPosition.LEFT) {
            // move to left park (strafe right)
            drive.goToPosition(-0.3, -0.3, -0.3, -0.3, 1372, "move backwards");
            drive.goToPosition(-0.3, 0.3, 0.3, -0.3 , 200, "strafe right");



        } else if (park.getPosition() == DetectionAlgorithmTest.ParkingPosition.CENTER) {
            // move to center park (don't move at all)
            drive.goToPosition(-0.3, 0.3, 0.3, -0.3 , 200, "strafe right");

        } else {
            // move to right park (strafe more left)
            drive.goToPosition(0.3, 0.3, 0.3, 0.3 , 1000, "move forward");
            drive.goToPosition(-0.3, 0.3, 0.3, -0.3 , 200, "strafe right");


        }


        score.setClawPosition(constants.getClawOpenPos());



    }

    public void scoreCone(int fl, int fr, int bl, int br) {

        // move arm max
        score.goToPosition(constants.getHeightHigh(), 0.85);
        //begin thread for maintaining height of slides
        cont.set(true);

        //move forward to pole
        drive.goToPosition(0.3, 0.3, 0.3, 0.3, drive.avgPosition(fl, fr, bl, br), "move to pole");


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
        drive.findTape();
        score.goToPosition(174, 0.7);
        score.grabConeAuto();


    }
    public void useCam() {

    }
}
