package org.firstinspires.ftc.teamcode.Auto;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
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
public class BlueLeftHigh extends LinearOpMode {
    MecanumDrive drive;
    ScoringSystem score;
    AtomicBoolean cont;
    Thread liftThread;
    DetectionAlgorithmTest detect;
    OpenCvWebcam webcam;
    BNO055IMU imu;

    @Override
    public void runOpMode() throws InterruptedException {
        //initializing imu and camera
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // seehe calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";

        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);


        detect = new DetectionAlgorithmTest(telemetry);
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
                webcam.startStreaming(320, 176, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addData("Camera Init Error", errorCode);
                telemetry.update();
            }
        });

        //TODO: Possibly change turns from encoder to IMU angles
        //TODO: Work on auto for all the side (make different methods for each side?)
        //thread for slides
        liftThread = new Thread(){
            @Override
            public void run(){
                // keeps the slides from sliding down on its own
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


        telemetry.addData("Status", "Initialized");
        telemetry.update();
        //close claw
        score.setClawPosition(0.24);
        waitForStart();
        webcam.stopStreaming();
        liftThread.start();
        blueLeft();
    }
    public void blueLeft(){
        telemetry.addData("liftPow", score.getPower());
        telemetry.addData("liftPos", score.getEncoderPosition());telemetry.update();
        //lift claw a little bit
        score.goToPosition(370, 0.7);
        sleep(200);
        // move forward a square and push sleeved cone out of the way
        drive.goToPosition(0.3,  0.3,  0.3, 0.3, avgPosition(1500, 1500, 1600, 1500), "forward");
        sleep(100);
        // move back a little from pushing the cone out of the way
        drive.goToPosition(-0.3, -0.3,  -0.3, -0.3, avgPosition(270, 300, 300, 300), "forward");

        //drive.turnToInitialPosition();
        //strafe right
        drive.goToPosition(0.4, -0.4, -0.4, 0.4, avgPosition(-927-927, 800+820, 900+1080, -905-705), "strafe right");

        // turn
        //drive.goToPosition(-0.3, 0.3, -0.3, 0.3, avgPosition(-311, 325, -345, 333), "turn to pole");
        sleep(100);
        // move arm max
        score.goToPosition(2340, 0.85);
        //begin thread for maintaining height of slides
        cont.set(true);
        //move forward closer to pole
        drive.goToPosition(0.3, 0.3, 0.3, 0.3, avgPosition(180, 250, 100, 100), "move to pole");
        sleep(1000);

        //lower cone onto pole
        score.goToPosition(score.getEncoderPosition()-300, 0.4);
        score.setClawPosition(0);
        sleep(300);
        score.setClawPosition(0.24);
        //move back from pole to strafe right
        drive.goToPosition(-0.3, -0.3, -0.3, -0.3, avgPosition(-130, -147, -191, -88), "move back from pole");
        cont.set(false);
        //moves slides down
        score.goToPosition(0, 0.3);
        sleep(300);
        //drive.turn(-Math.PI/2, 0.3);
        //moves robot to correct parking position
        if (detect.getPosition() == DetectionAlgorithmTest.ParkingPosition.LEFT) {
            // move to left
            drive.goToPosition(-0.5, 0.5, 0.5, -0.5, avgPosition(-5007,2941,3226,-3036), "strafe left (more left)");
            drive.goToPosition(0.3, 0.3, 0.3, 0.3, avgPosition(600,600,600,650), "strafe right");
            //drive.goToPosition(-0.3, -0.3, -0.3, -0.3, 3410, "strafe right");
            //drive.goToPosition(-0.3, 0.3, 0.3, -0.3, 300, "strafe right");

        } else if (detect.getPosition() == DetectionAlgorithmTest.ParkingPosition.CENTER) {
            // move to center
            drive.goToPosition(-0.5, 0.5, 0.5, -0.5, avgPosition(-1759,1748,1937,-1784), "strafe left (center)");
            drive.goToPosition(0.3, 0.3, 0.3, 0.3, avgPosition(400,400,400,400), "strafe right");
            //drive.goToPosition(-0.3, -0.3, -0.3, -0.3, 1530, "strafe right");
            //drive.turnToInitialPosition();
        } else {
            // move to right
            drive.goToPosition(-0.3, 0.3, 0.3, -0.3, avgPosition(-560,565,642,-585), "strafe left");
            drive.goToPosition(0.3, 0.3, 0.3, 0.3, avgPosition(400,400,400,400), "strafe right");
            //drive.goToPosition(-0.3, -0.3, -0.3, -0.3, 400, "strafe right");

        }
        //drive.turnToInitialPosition();
        
        score.setClawPosition(0);

    }
    public int avgPosition(int fl, int fr, int bl, int br){
        return (int)(Math.abs(fl) + Math.abs(fr) + Math.abs(bl) + Math.abs(br))/4;
    }

}
