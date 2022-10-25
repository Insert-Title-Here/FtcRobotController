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
//TODO:Needs testing
@Autonomous
public class BlueLeftSpecial extends LinearOpMode {
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

            // Camera checks sleeve...stores parking location??

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

        blueLeftDefense();
    }
    public void blueLeftDefense(){
        //close claw
        score.setClawPosition(0.47);
        sleep(1000);
        //lift claw a little bit
        score.goToPosition(100, 0.7);
        sleep(200);
        // move forward 3 squares
        drive.goToPosition(0.3, 0.3,  0.3, 0.3, avgPosition(2307, 2203, 2230, 2313), "forward");

        //strafe right
        drive.goToPosition(0.3, -0.3, -0.3, 0.3, avgPosition(758, -700, -714, 816), "strafe right");
        sleep(100);
        // turn
        //drive.goToPosition(-0.3, 0.3, -0.3, 0.3, avgPosition(-311, 325, -345, 333), "turn to pole");

        // move arm max
        score.goToPosition(2340, 0.85);
        cont.set(true);
        drive.goToPosition(0.3, 0.3, 0.3, 0.3, avgPosition(291, 260, 277, 263), "move to pole");
        sleep(100);
        score.setClawPosition(1);
        sleep(200);
        score.setClawPosition(0.47);
        sleep(200);
        drive.goToPosition(-0.3, -0.3, -0.3, -0.3, avgPosition(-200, -197, -211, -298), "move back from pole");
        // lowers arm after scoring first cone
        cont.set(false);
        score.goToPosition(1000, 0.5);
        sleep(200);
        drive.goToPosition(0.3,0.3,0.3,0.3,avgPosition(386,398,367,371),"Drive forward");
        sleep(15000);
        drive.goToPosition(-0.3,-0.3,-0.3,-0.3, avgPosition(-400,-420,-450,-450),"drive backward");
        score.goToPosition(0, 0.3);

        if (detect.getPosition() == DetectionAlgorithmTest.ParkingPosition.LEFT) {
            // move to left
            drive.goToPosition(-0.3, 0.3, 0.3, -0.3, avgPosition(-1823,1783,1921,-1812), "strafe more left");
        } else if (detect.getPosition() == DetectionAlgorithmTest.ParkingPosition.CENTER) {
            // move to center
            drive.goToPosition(-0.3, 0.3, 0.3,-0.3, avgPosition(-597,587,641,-592),"strafe left (center)");
        } else {
            // move to right
            drive.goToPosition(0.3, -0.3, -0.3, 0.3, avgPosition(582,-629,-611,641), "strafe right");

        }

        //1 (far left) (general code)
        //drive.goToPosition(0.3, -0.3, -0.3, 0.3, avgPosition(750,-750,-750,750), "strafe right");
        //cont.set(false);





        //2 middle
        //3 far right
        //drive.goToPosition(0.3, -0.3, -0.3, 0.3, avgPosition(1152, -1177, -1164, 1196), "strafe right");

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
