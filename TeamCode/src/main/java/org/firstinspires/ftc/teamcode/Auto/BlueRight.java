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
public class BlueRight extends LinearOpMode {
    MecanumDrive drive;
    ScoringSystem score;
    //OpenCvWebcam webcam;
    AtomicBoolean cont;
    Thread liftThread;
    String parkLocation;
    DetectionAlgorithm detect;
    OpenCvWebcam webcam;

    @Override
    public void runOpMode() throws InterruptedException {
        detect = new DetectionAlgorithm(telemetry);
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

        waitForStart();
        webcam.stopStreaming();
        liftThread.start();

        //TODO: Consider whethere or not to just have the code laid out here or by just calling the method
        //If changed above, then must do for all
        blueRight();
    }
    public void blueRight(){

        //close claw
        score.setClawPosition(0.45);
        sleep(1000);
        //lift claw a little bit
        score.goToPosition(100, 0.7);
        sleep(200);
        // move forward a square
        drive.goToPosition(0.3, 0.3,  0.3, 0.3, avgPosition(1100, 1100, 1088, 1066), "forward");

        //strafe left
        drive.goToPosition(-0.3, 0.3, 0.3, -0.3, avgPosition(-1954, 1686, 1820, -1987), "strafe right");
        sleep(1000);
        // turn
        //drive.goToPosition(-0.3, 0.3, -0.3, 0.3, avgPosition(-311, 325, -345, 333), "turn to pole");

        // move arm max
        score.goToPosition(2380, 0.85);
        cont.set(true);
        drive.goToPosition(0.3, 0.3, 0.3, 0.3, avgPosition(95, 100, 98, 87), "move to pole");
        sleep(1200);


        score.setClawPosition(0.9);
        sleep(1000);
        drive.goToPosition(-0.3, -0.3, -0.3, -0.3, avgPosition(-100, -97, -111, -98), "move back from pole");
        // lowers arm after scoring first cone
        cont.set(false);
        score.goToPosition(0, 0.3);
        sleep(1000);

        //1 (far left) (general code)
        // drive.goToPosition(-0.3, -0.3, -0.3, -0.3, avgPosition(-498, -506, -557, -565), "move back further from pole");
        // sleep(50);
        //turn right a little(straighten out)
        // drive.goToPosition(0.3, -0.3, 0.3, -0.3, avgPosition(311, -325, 345, -333), "turn straight");
        //sleep(50);
        //drive forward a little
        //drive.goToPosition(0.3,0.3,0.3,0.3,avgPosition(310, 380, 320, 290), "drive forward a little");
        if (detect.getPosition() == DetectionAlgorithm.ParkingPosition.LEFT) {
            // move to left
            drive.goToPosition(0.3, -0.3, -0.3, 0.3, avgPosition(750,-750,-750,750), "strafe right");
        } else if (detect.getPosition() == DetectionAlgorithm.ParkingPosition.CENTER) {
            // move to center TODO: measure drive encoder values
            drive.goToPosition(0.3, -0.3, -0.3, 0.3, avgPosition(750,-750,-750,750), "strafe right (center)");
        } else {
            // move to right TODO: measure drive encoder values
            drive.goToPosition(0.3, -0.3, -0.3, 0.3, avgPosition(750,-750,-750,750), "strafe right (center)");

        }





        /*
        //2 middle
        drive.goToPosition(0.3, -0.3, -0.3, 0.3, avgPosition(1267, -1251, -1246, 304), "strafe right");
        //3 far right
        drive.goToPosition(0.3, -0.3, -0.3, 0.3, avgPosition(1152, -1177, -1164, 1196), "strafe right");

    */
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
