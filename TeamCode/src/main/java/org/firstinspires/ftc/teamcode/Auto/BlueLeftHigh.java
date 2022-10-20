package org.firstinspires.ftc.teamcode.Auto;

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

    @Override
    public void runOpMode() throws InterruptedException {
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



        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();
        webcam.stopStreaming();
        liftThread.start();
        blueLeft();
    }
    public void blueLeft(){
        //close claw
        score.setClawPosition(0.47);
        sleep(800);
        //lift claw a little bit
        score.goToPosition(100, 0.7);
        sleep(200);
        // move forward a square
        drive.goToPosition(0.3, 0.3,  0.3, 0.3, avgPosition(1250, 1250, 1200, 1236), "forward");

        //strafe right
        drive.goToPosition(0.3, -0.3, -0.3, 0.3, avgPosition(1854, -1686, -1820, 1947), "strafe right");
        sleep(500);
        // turn
        //drive.goToPosition(-0.3, 0.3, -0.3, 0.3, avgPosition(-311, 325, -345, 333), "turn to pole");

        // move arm max
        score.goToPosition(2340, 0.85);
        cont.set(true);
        drive.goToPosition(0.3, 0.3, 0.3, 0.3, avgPosition(95, 100, 98, 87), "move to pole");
        sleep(2000);
        score.setClawPosition(1);
        sleep(300);
        score.setClawPosition(0.47);
        drive.goToPosition(-0.3, -0.3, -0.3, -0.3, avgPosition(-100, -97, -111, -98), "move back from pole");
        // lowers arm after scoring first cone
        cont.set(false);
        score.goToPosition(0, 0.3);
        sleep(300);


        if (detect.getPosition() == DetectionAlgorithmTest.ParkingPosition.LEFT) {
            // move to left
            drive.goToPosition(-0.3, 0.3, 0.3, -0.3, avgPosition(-3007,2941,3226,-3036), "strafe left (more left)");
        } else if (detect.getPosition() == DetectionAlgorithmTest.ParkingPosition.CENTER) {
            // move to center TODO: measure drive encoder values
            drive.goToPosition(-0.3, 0.3, 0.3, -0.3, avgPosition(-1759,1748,1937,-1784), "strafe left (center)");
        } else {
            // move to right TODO: measure drive encoder values
            drive.goToPosition(-0.3, 0.3, 0.3, -0.3, avgPosition(-560,565,642,-585), "strafe left");

        }
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
        score.setClawPosition(1);
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
