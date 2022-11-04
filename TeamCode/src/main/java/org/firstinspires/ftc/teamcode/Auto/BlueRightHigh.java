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
public class BlueRightHigh extends LinearOpMode {
    MecanumDrive drive;
    ScoringSystem score;
    AtomicBoolean cont;
    Thread liftThread;
    String parkLocation;
    DetectionAlgorithmTest detect;
    OpenCvWebcam webcam;
    BNO055IMU imu;
    double radius;

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

        telemetry.addData("position", parkLocation);
        telemetry.addData("Status", "Initialized");
        telemetry.addData("liftPos", score.getEncoderPosition());
        telemetry.addData("clawPos", score.getClawPosition());
        telemetry.addData("liftPow", score.getPower());

        telemetry.update();


        //TODO: Possibly change turns from encoder to IMU angles
        //TODO: Work on auto for all the side (make different methods for each side?)

        //thread for slides
        liftThread = new Thread(){
            @Override
            public void run(){
                while(opModeIsActive()){
                    // keeps the slides from sliding down on its own
                    if((score.getEncoderPosition() > 1200 && cont.get())){
                        score.setPower(0.1);
                    }
                    telemetry.addData("liftPow", score.getPower());
                    telemetry.addData("liftPos", score.getEncoderPosition());
                    telemetry.addData("anglePos", imu.getAngularOrientation().firstAngle);
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
        blueRight();
    }
    public void blueRight(){
        telemetry.addData("liftPow", score.getPower());
        telemetry.addData("liftPos", score.getEncoderPosition());
        telemetry.addData("anglePos", imu.getAngularOrientation().firstAngle);
        telemetry.update();
        //lift claw a little bit
        score.goToPosition(150, 0.7);
        sleep(200);
        // move forward a square and push sleeved cone out of the way
        drive.goToPosition(0.3,  0.3,  0.3, 0.3, avgPosition(1300, 1300, 1359, 1200), "forward");
        sleep(100);
        // move back a little from pushing the cone out of the way
        drive.goToPosition(-0.3, -0.3,  -0.3, -0.3, avgPosition(100, 100, 100, 100), "forward");

        //drive.turnToInitialPosition();
        //strafe left
        drive.goToPosition(-0.4, 0.4, 0.4, -0.4, avgPosition(-827-827, 860+820, 1000+1080, -955-855), "strafe left");

        // turn
        //drive.goToPosition(-0.3, 0.3, -0.3, 0.3, avgPosition(-311, 325, -345, 333), "turn to pole");
        sleep(100);
        // move arm to max
        score.goToPosition(2340, 0.85);
        //begin thread for maintaining height of slides
        cont.set(true);
        //move forward closer to pole
        drive.goToPosition(0.3, 0.3, 0.3, 0.3, avgPosition(100, 100, 100, 150), "move to pole");
        sleep(1000);

        //lower cone onto pole
        score.goToPosition(score.getEncoderPosition()-300, 0.4);
        score.setClawPosition(0);
        sleep(300);
        score.setClawPosition(0.24);
        //move back from pole to strafe right
        drive.goToPosition(-0.3, -0.3, -0.3, -0.3, avgPosition(-50, -97, -111, -98), "move back from pole");
        cont.set(false);
        //moves slides down
        score.goToPosition(0, 0.3);
        sleep(300);
        //moves robot to correct parking position
        if (detect.getPosition() == DetectionAlgorithmTest.ParkingPosition.LEFT) {
            // move to left
            drive.goToPosition(0.3, -0.3, -0.3, 0.3, avgPosition(750,-700,-600,500), "strafe right");
            drive.goToPosition(0.3, 0.3, 0.3, 0.3, avgPosition(400,400,400,400), "strafe right"  );

        } else if (detect.getPosition() == DetectionAlgorithmTest.ParkingPosition.CENTER) {
            // move to center
            drive.goToPosition(0.3, -0.3, -0.3, 0.3, avgPosition(892+892,-900-900,-905-905,900+900), "strafe right (center)");
            drive.goToPosition(0.3, 0.3, 0.3, 0.3, avgPosition(400,400,400,400), "strafe right");

        } else {
            // move to right
            drive.goToPosition(0.5, -0.5, -0.5, 0.5, avgPosition(2017+2017,-1559-1559,-1557-1557,1613+1613), "strafe right (more right)");
            drive.goToPosition(0.3, 0.3, 0.3, 0.3, avgPosition(400,400,400,400), "strafe right");

        }





        /*
        //2 middle
        drive.goToPosition(0.3, -0.3, -0.3, 0.3, avgPosition(1267, -1251, -1246, 304), "strafe right");
        //3 far right
        drive.goToPosition(0.3, -0.3, -0.3, 0.3, avgPosition(1152, -1177, -1164, 1196), "strafe right");
    */
        score.setClawPosition(0);
    }

    public int avgPosition(int fl, int fr, int bl, int br){
        return (int)(Math.abs(fl) + Math.abs(fr) + Math.abs(bl) + Math.abs(br))/4;
    }


}
