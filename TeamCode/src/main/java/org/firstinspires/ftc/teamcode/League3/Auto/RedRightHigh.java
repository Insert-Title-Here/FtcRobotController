package org.firstinspires.ftc.teamcode.League3.Auto;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.League3.Common.Constants;
import org.firstinspires.ftc.teamcode.League3.Common.MecanumDrive;
import org.firstinspires.ftc.teamcode.League3.Common.ScoringSystem;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.concurrent.atomic.AtomicBoolean;
//@Autonomous
public class RedRightHigh extends LinearOpMode {
    // instantiations
    MecanumDrive drive;
    ScoringSystem score;
    AtomicBoolean cont;
    Thread liftThread;
    String parkLocation;
    DetectionAlgorithmRight detect;
    OpenCvWebcam webcam;
    BNO055IMU imu;
    Constants constants;
    double radius;

    @Override
    public void runOpMode() throws InterruptedException {

        // imu instantiation etc.
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // seehe calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";

        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu = hardwareMap.get(BNO055IMU.class, "imu");

        // initializations
//        imu.initialize(parameters);
        detect = new DetectionAlgorithmRight(telemetry);
        drive = new MecanumDrive(hardwareMap, telemetry);
        score = new ScoringSystem(hardwareMap, telemetry);
        cont = new AtomicBoolean();
        cont.set(false);
        constants = new Constants();

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

        telemetry.addData("position", parkLocation);
        telemetry.addData("Status", "Initialized");
        telemetry.addData("liftPos", score.getEncoderPosition());
        telemetry.addData("clawPos", score.getClawPosition());
        telemetry.addData("liftPow", score.getPower());

        telemetry.update();

        //TODO: Possibly change turns from encoder to IMU angles

        // thread from keeping the slide from going down
        liftThread = new Thread(){
            @Override
            public void run(){
                while(opModeIsActive()){
                    if((score.getEncoderPosition() > constants.getHeightLow()-50 && cont.get())){
                        score.setPower(constants.getSteadyPow());
                    }


                    telemetry.addData("liftPow", score.getPower());
                    telemetry.addData("liftPos", score.getEncoderPosition());
//                    telemetry.addData("anglePos", imu.getAngularOrientation().firstAngle);
                    telemetry.update();
                }

            }
        };
        //TRY SETTING THE COMMANDS INSIDE THE THREAD AND SEE IF IT WORKS THAT WAY

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        //close claw
        score.setClawPosition(constants.getClawClosePos());
        waitForStart();
        webcam.stopStreaming();
        liftThread.start();

        //TODO: Consider whethere or not to just have the code laid out here or by just calling the method
        //If changed above, then must do for all
        redRight();
    }
    public void redRight(){
        telemetry.addData("liftPow", score.getPower());
        telemetry.addData("liftPos", score.getEncoderPosition());
//        telemetry.addData("anglePos", imu.getAngularOrientation().firstAngle);
        telemetry.update();
        //lift claw a little bit
        score.goToPosition(270, 0.7);
        sleep(200);
        // move forward a bit more than a square (pushes sleeve out of way)
        drive.goToPosition(0.3,  0.3,  0.3, 0.3, avgPosition(1300, 1300, 1300, 1200), "forward");
        sleep(100);
        // moves back to center-ish of square
        drive.goToPosition(-0.3, -0.3,  -0.3, -0.3, avgPosition(100, 100, 50, 50), "backwards");

        //drive.turnToInitialPosition();

        //strafe left to in front of highest pole
        drive.goToPosition(-0.4, 0.4, 0.4, -0.4, avgPosition(-927-927, 800+870, 1000+1000, -805-805), "strafe left");

        // turn
        //drive.goToPosition(-0.3, 0.3, -0.3, 0.3, avgPosition(-311, 325, -345, 333), "turn to pole");
        sleep(100);
        // move arm max
        score.goToPosition(constants.getHeightHigh(), 1);
        cont.set(true);
        // drive to cone
        drive.goToPosition(0.3, 0.3, 0.3, 0.3, avgPosition(160, 180, 98, 130), "move to pole");
        // wait for slide to stop shaking
        sleep(1000);


        // slide goes down 300 tics from it's original position
        score.goToPosition(score.getEncoderPosition()-300, 0.4);
        // cone released
        score.setClawPosition(constants.getClawOpenPos());
        sleep(300);

        // drive backwards to center-ish of squares (y direction)
        drive.goToPosition(-0.3, -0.3, -0.3, -0.3, avgPosition(-130, -97, -201, -98), "move back from pole");

        cont.set(false);
        // lowers arm after scoring first cone
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
        if (detect.getPosition() == DetectionAlgorithmRight.ParkingPosition.LEFT) {
            // move to left
            drive.goToPosition(0.3, -0.3, -0.3, 0.3, avgPosition(750,-650,-650,600), "strafe right");
            // move forward a bit to guarantee park
            drive.goToPosition(0.3, 0.3, 0.3, 0.3, avgPosition(400,400,400,400), "strafe right");

        } else if (detect.getPosition() == DetectionAlgorithmRight.ParkingPosition.CENTER) {
            // move to center
            drive.goToPosition(0.3, -0.3, -0.3, 0.3, avgPosition(892+892,-910-910,-905-905,928+928), "strafe right (center)");
            // move forward a bit to guarantee park
            drive.goToPosition(0.3, 0.3, 0.3, 0.3, avgPosition(400,400,400,400), "strafe right");

        } else {
            // move to right
            drive.goToPosition(0.5, -0.5, -0.5, 0.5, avgPosition(2017+2017,-1559-1559,-1557-1557,1613+1613), "strafe right (more right)");
            // move forward a bit to guarantee park
            drive.goToPosition(0.3, 0.3, 0.3, 0.3, avgPosition(400,400,400,400), "strafe right");

        }





        /*
        //2 middle
        drive.goToPosition(0.3, -0.3, -0.3, 0.3, avgPosition(1267, -1251, -1246, 304), "strafe right");
        //3 far right
        drive.goToPosition(0.3, -0.3, -0.3, 0.3, avgPosition(1152, -1177, -1164, 1196), "strafe right");
    */
        score.setClawPosition(constants.getClawClosePos());
    }
    // returns the average tics for mecanum wheels
    public int avgPosition(int fl, int fr, int bl, int br){
        return (int)(Math.abs(fl) + Math.abs(fr) /*+ Math.abs(bl)*/ + Math.abs(br))/3;
    }

}
