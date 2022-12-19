package org.firstinspires.ftc.teamcode.State.Auto;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.State.Common.Constants;
import org.firstinspires.ftc.teamcode.State.Common.MecanumDrive;
import org.firstinspires.ftc.teamcode.State.Common.ScoringSystem;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.concurrent.atomic.AtomicBoolean;
//@Autonomous
public class RedLeftHigh extends LinearOpMode {
    // instantiations
    MecanumDrive drive;
    ScoringSystem score;
    AtomicBoolean cont;
    Thread liftThread;
    DetectionAlgorithmRight detect;
    OpenCvWebcam webcam;
    Constants constants;
    BNO055IMU imu;

    @Override
    public void runOpMode() throws InterruptedException {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // seehe calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";

        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);


        // initializations
        detect = new DetectionAlgorithmRight(telemetry);
        drive = new MecanumDrive(hardwareMap, telemetry);
        score = new ScoringSystem(hardwareMap, telemetry);
        cont = new AtomicBoolean();
        cont.set(false);

        // camera init
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

        // thread for keeping slide from going down
        liftThread = new Thread(){
            @Override
            public void run(){
                while(opModeIsActive()){
                    if((score.getEncoderPosition() > constants.getHeightLow()-50 && cont.get())){
                        score.setPower(constants.getSteadyPow());
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
        redLeft();
    }
    public void redLeft(){
        telemetry.addData("liftPow", score.getPower());
        telemetry.addData("liftPos", score.getEncoderPosition());telemetry.update();
        telemetry.update();
        //lift claw a little bit
        score.goToPosition(270, 0.7);
        sleep(200);
        // move forward a bit more than a square (pushes sleeve out of way)
        drive.goToPosition(0.3,  0.3,  0.3, 0.3, avgPosition(1500, 1500, 1400, 1300), "forward");
        sleep(100);
        // moves back to center-ish of square
        drive.goToPosition(-0.3, -0.3,  -0.3, -0.3, avgPosition(300, 300, 200, 100), "forward");

        //drive.turnToInitialPosition();
        //strafe right to in front of highest pole
        drive.goToPosition(0.4, -0.4, -0.4, 0.4, avgPosition(-927-927, 980+900, 700+880, -805-755), "strafe right");

        // turn
        //drive.goToPosition(-0.3, 0.3, -0.3, 0.3, avgPosition(-311, 325, -345, 333), "turn to pole");
        sleep(100);
        // move arm max
        score.goToPosition(constants.getHeightHigh(), 0.85);
        cont.set(true);
        // drive to cone
        drive.goToPosition(0.3, 0.3, 0.3, 0.3, avgPosition(100, 100, 140, 100), "move to pole");
        // wait for slide to stop shaking
        sleep(1000);

        // slide goes down 300 tics from it's original position
        score.goToPosition(score.getEncoderPosition()-300, 0.4);
        // cone released
        score.setClawPosition(constants.getClawOpenPos());
        sleep(300);

        // drive backwards to center-ish of squares (y direction)
        drive.goToPosition(-0.3, -0.3, -0.3, -0.3, avgPosition(-160, -117, -111, -98), "move back from pole");
        cont.set(false);
        // lowers arm after scoring first cone
        score.goToPosition(0, 0.3);
        sleep(300);
//        drive.turn(-Math.PI/2, 0.3);

        // parking
        if (detect.getPosition() == DetectionAlgorithmRight.ParkingPosition.LEFT) {
            // move to left
            drive.goToPosition(-0.5, 0.5, 0.5, -0.5, avgPosition(-5007,2941,3226,-3036), "strafe left (more left)");
            // move forward a bit to guarantee park
            drive.goToPosition(0.3, 0.3, 0.3, 0.3, avgPosition(600,600,600,650), "strafe right");
//            drive.goToPosition(-0.3, -0.3, -0.3, -0.3, 3410, "strafe right");
//            drive.goToPosition(-0.3, 0.3, 0.3, -0.3, 300, "strafe right");

        } else if (detect.getPosition() == DetectionAlgorithmRight.ParkingPosition.CENTER) {
            // move to center
            drive.goToPosition(-0.5, 0.5, 0.5, -0.5, avgPosition(-1759,1648,1737,-1784), "strafe left (center)");
            // move forward a bit to guarantee park
            drive.goToPosition(0.3, 0.3, 0.3, 0.3, avgPosition(400,400,400,400), "strafe right");
//            drive.goToPosition(-0.3, -0.3, -0.3, -0.3, 1530, "strafe right");
//            drive.turnToInitialPosition();
        } else {
            // move to right
            drive.goToPosition(-0.3, 0.3, 0.3, -0.3, avgPosition(-560,565,542,-585), "strafe left");
            // move forward a bit to guarantee park
            drive.goToPosition(0.3, 0.3, 0.3, 0.3, avgPosition(400,400,400,400), "strafe right");
//            drive.goToPosition(-0.3, -0.3, -0.3, -0.3, 400, "strafe right");


        }
        //drive.turnToInitialPosition();

        /*
        //1 (far right) (general code)
        drive.goToPosition(-0.3, -0.3, -0.3, -0.3, avgPosition(-498, -506, -557, -565), "move back further from pole");
        sleep(0);
        //turn left a little (straighten out)
        drive.goToPosition(-0.3, 0.3, -0.3, 0.3, avgPosition(-271, 280, -260, 290), "turn straight");
        sleep(50);
        //drive forward a little
        drive.goToPosition(0.3,0.3,0.3,0.3,avgPosition(310, 380, 320, 290), "drive forward a little");




        //2
        drive.goToPosition(-0.3, 0.3, 0.3, -0.3, avgPosition(-1267, 1251, 1246, -304), "strafe left");
        //3
        drive.goToPosition(-0.3, 0.3, 0.3, -0.3, avgPosition(-1152, 1177, 1164, -1196), "strafe left");

    */
        score.setClawPosition(constants.getClawOpenPos());

    }
    // returns the average tics for mecanum wheels
    public int avgPosition(int fl, int fr, int bl, int br){
        return (int)(Math.abs(fl) + Math.abs(fr) /*+ Math.abs(bl)*/ + Math.abs(br))/3;
    }

}
