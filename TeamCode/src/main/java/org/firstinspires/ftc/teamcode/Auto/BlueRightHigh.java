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
        detect = new DetectionAlgorithmTest(telemetry);
        drive = new MecanumDrive(hardwareMap, telemetry);
        score = new ScoringSystem(hardwareMap);
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
        //close claw
        score.setClawPosition(0.24);
        waitForStart();
        webcam.stopStreaming();
        liftThread.start();

        //TODO: Consider whethere or not to just have the code laid out here or by just calling the method
        //If changed above, then must do for all
        blueRight();
    }
    public void blueRight(){

        //lift claw a little bit
        score.goToPosition(100, 0.7);
        sleep(200);
        // move forward a square
        drive.goToPosition(0.4, 0.4,  0.4, 0.4, avgPosition(1000, 1000, 1059, 1000), "forward");
        //drive.turnToInitialPosition();
        //strafe left
        drive.goToPosition(-0.4, 0.4, 0.4, -0.4, avgPosition(-1854, 1640, 2160, -1710), "strafe left");
        sleep(500);
        // turn
        //drive.goToPosition(-0.3, 0.3, -0.3, 0.3, avgPosition(-311, 325, -345, 333), "turn to pole");

        // move arm max
        score.goToPosition(2340, 0.85);
        cont.set(true);
        drive.goToPosition(0.3, 0.3, 0.3, 0.3, avgPosition(90, 80, 98, 100), "move to pole");
        sleep(1000);


        score.goToPosition(score.getEncoderPosition()-300, 0.4);
        score.setClawPosition(0);
        sleep(300);
        score.setClawPosition(0.24);
        drive.goToPosition(-0.3, -0.3, -0.3, -0.3, avgPosition(-30, -97, -111, -98), "move back from pole");
        // lowers arm after scoring first cone
        cont.set(false);
        score.goToPosition(0, 0.3);
        sleep(300);

        //1 (far left) (general code)
        // drive.goToPosition(-0.3, -0.3, -0.3, -0.3, avgPosition(-498, -506, -557, -565), "move back further from pole");
        // sleep(50);
        //turn right a little(straighten out)
        // drive.goToPosition(0.3, -0.3, 0.3, -0.3, avgPosition(311, -325, 345, -333), "turn straight");
        //sleep(50);
        //drive forward a little
        //drive.goToPosition(0.3,0.3,0.3,0.3,avgPosition(310, 380, 320, 290), "drive forward a little");
        if (detect.getPosition() == DetectionAlgorithmTest.ParkingPosition.LEFT) {
            // move to left
            drive.goToPosition(0.3, -0.3, -0.3, 0.3, avgPosition(750,-750,-750,600), "strafe right");
        } else if (detect.getPosition() == DetectionAlgorithmTest.ParkingPosition.CENTER) {
            // move to center
            drive.goToPosition(0.3, -0.3, -0.3, 0.3, avgPosition(1784,-1820,-1811,1856), "strafe right (center)");
        } else {
            // move to right
            drive.goToPosition(0.3, -0.3, -0.3, 0.3, avgPosition(4035,-3117,-3114,3226), "strafe right (more right)");

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
    /*
    //TODO: check if camera angle works
    private class DetectionAlgorithm extends OpenCvPipeline {
        Mat original;
        Mat changed;

        @Override
        public Mat processFrame(Mat input) {
            input.copyTo(original);
            changed = new Mat();
            if(original.empty()) {
                return input;
            }
            // cyan magenta yellow
            Imgproc.cvtColor(original, changed, Imgproc.COLOR_RGB2YCrCb);
            // magenta 255, 0, 255
            Core.inRange(changed, new Scalar(240, 0 ,240), new Scalar(255, 0, 255), changed);
            return null;
        }


    }
    */

}
