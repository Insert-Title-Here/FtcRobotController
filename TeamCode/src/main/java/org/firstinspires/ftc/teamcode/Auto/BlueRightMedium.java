package org.firstinspires.ftc.teamcode.Auto;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Common.MecanumDrive;
import org.firstinspires.ftc.teamcode.Common.ScoringSystem;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;
import java.util.concurrent.atomic.AtomicBoolean;
//@Autonomous
public class BlueRightMedium extends LinearOpMode {
    MecanumDrive drive;
    ScoringSystem score;
    //OpenCvWebcam webcam;
    AtomicBoolean cont;
    Thread liftThread;
    String parkLocation;
    DetectionAlgorithmRight detect;
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

        detect = new DetectionAlgorithmRight(telemetry);
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
        score.setClawPosition(0.24);
        waitForStart();
        webcam.stopStreaming();
        liftThread.start();
        //TODO: Consider whethere or not to just have the code laid out here or by just calling the method
        //If changed above, then must do for all
        blueRight();
    }
    public void blueRight() throws InterruptedException {

        //close claw
        score.setClawPosition(0.24);
        sleep(800);
        //lift claw a little bit
        score.goToPosition(100, 0.7);
        sleep(200);
        // move forward a square
        drive.goToPosition(0.3, 0.3,  0.3, 0.3, avgPosition(1346, 1263, 1321, 1206), "forward");
        // turning to left --> -540, 527, -577, 566
        double radians = 3.14159 / 4;
        drive.turn(radians);
        sleep(500);
        // turn
        //drive.goToPosition(-0.3, 0.3, -0.3, 0.3, avgPosition(-311, 325, -345, 333), "turn to pole");
        // move arm up
        score.goToPosition(1660, 0.85);
        cont.set(true);
        // move to pole
        drive.goToPosition(-0.3, 0.3, 0.3, -0.3, avgPosition(-558, 590, 600, -559), "forward");
        sleep(1000);
        score.goToPosition(score.getEncoderPosition()-300, 0.4);
        score.setClawPosition(0);
        sleep(300);
        drive.goToPosition(-0.3, -0.3, -0.3, -0.3, avgPosition(-558, 590, 600, -559), "move back from pole");
        // lowers arm after scoring first cone
        cont.set(false);
        score.goToPosition(0, 0.3);
        radians = -3.14159 / 4;
        // turns back forward
        drive.turn(radians);
        // moves forward 1 square (perpendicular to cone stack)
        drive.goToPosition(0.3, 0.3, 0.3, 0.3, avgPosition(980, 950, 1016, 937), "move forward a square");
        // turn toward cone stack (90 degrees)
        radians = -3.14159 / 2;
        drive.turn(radians);
        // position of highest cone in stack
        score.goToPosition(320, 0.4);
        // color sensor movement forward (tape) if not using encoder based
        drive.goToPosition(0.3, 0.3, 0.3, 0.3, avgPosition(900, 900, 1016, 1000), "move forward a square");
        // grab and lift
        score.setClawPosition(0);
        score.goToPosition(600, 0.4);
        //move backwards a bit
        drive.goToPosition(-0.4, -0.4, -0.4, -0.4, avgPosition(-500, -500, -500, -500), "move backwards a bit");
        // turn 180
        drive.turn(3.14159);
//        drive.goToPosition(0.3, 0.3, 0.3, 0.3, avgPosition(900, 900, 1016, 1000), "move forward a square");

        // move forward a bit more
        drive.findTape();
        //go forward until...
        while (!score.grabCone()) {
            drive.goToPosition(0.4, 0.4, 0.4, 0.4);
        }
        // stop driving
        drive.setPower(0, 0, 0, 0);
        // grab cone
        score.setClawPosition(0.24);
        // lift up
        score.goToPosition(600, 0.6);
        // backup
        drive.goToPosition(-0.4, -0.4, -0.4, -0.4, 200, "backwards");
        //turn
        radians = 3.14 * 7 / 6;
        drive.turn(radians); // TODO: make sure drive.turn works
        // TODO: implement contours...find pole & distance from pole
        // move forward some
        drive.goToPosition(0.3, 0.3, 0.3, 0.3, avgPosition(500, 500, 500, 500), "move forward some");
        // turn left towards medium cone
        radians = 3.14159 / 4;
        drive.turn(radians);
        // move arm up
        score.goToPosition(1660, 0.85);
        cont.set(true);
        // move to pole
        drive.goToPosition(-0.3, 0.3, 0.3, -0.3, avgPosition(-558, 590, 600, -559), "forward");
        sleep(1000);
        score.goToPosition(score.getEncoderPosition()-300, 0.4);
        score.setClawPosition(0);
        sleep(300);
        // move back from pole
        drive.goToPosition(-0.3, -0.3, -0.3, -0.3, avgPosition(-558, 590, 600, -559), "move back from pole");
        // lowers arm after scoring first cone
        cont.set(false);
        score.goToPosition(0, 0.3);
        //turn
        radians = -3.14159 / 4;
        drive.turn(radians);
        //park (only have to move forward or backwards...currently in center position (cyan)
        if (detect.getPosition() == DetectionAlgorithmRight.ParkingPosition.LEFT) {
            // move to left TODO: measure drive encoder values
            drive.goToPosition(0.3, 0.3, -0.3, 0.3, avgPosition(1000,1000,900,900), "move forward");
        } else if (detect.getPosition() == DetectionAlgorithmRight.ParkingPosition.CENTER) {
            // move to center TODO: measure drive encoder values
//            drive.goToPosition(0.3, -0.3, -0.3, 0.3, avgPosition(1784,-1820,-1811,1856), "strafe right (center)");
        } else {
            // move to right TODO: measure drive encoder values
            drive.goToPosition(-0.3, -0.3, -0.3, -0.3, avgPosition(-1000,-1000,-1000,1000), "move backwards");
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