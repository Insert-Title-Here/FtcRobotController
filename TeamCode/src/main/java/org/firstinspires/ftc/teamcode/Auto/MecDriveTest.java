package org.firstinspires.ftc.teamcode.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Common.MecanumDrive;
import org.firstinspires.ftc.teamcode.Common.ScoringSystem;
/*
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

 */

@Autonomous
public class MecDriveTest extends LinearOpMode {
    MecanumDrive drive;
    ScoringSystem score;
    //OpenCvWebcam webcam;
    Thread liftThread;
    int parkLocation;
    @Override
    public void runOpMode() throws InterruptedException {
        drive = new MecanumDrive(hardwareMap, telemetry);
        score = new ScoringSystem(hardwareMap);
        /*
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "webcam"), cameraMonitorViewId);
        telemetry.addData("Status", "Initialized");
        telemetry.update();
         */





        // Camera checks sleeve...stores parking location??

        //TODO: Work on threading
        //TODO: Possibly change turns from encoder to IMU angles
        //TODO: Work on auto for all the side (make different methods for each side?)

        liftThread = new Thread(){
            @Override
            public void run(){
                while(true){
                    if(score.getEncoderPosition() > 1200){
                        //TODO
                        score.setPower(5);
                    }
                }

            }
        };



        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();
        liftThread.start();

        // POV is from the middle blue triangle side or the red middle triangle
        blueRight();
        //blueLeft();
        //redRight();
        //redLeft();





    }
    public int avgPosition(int fl, int fr, int bl, int br){
        return (int)(Math.abs(fl) + Math.abs(fr) + Math.abs(bl) + Math.abs(br))/4;
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
        score.goToPosition(2380, 0.5);
        drive.goToPosition(0.3, 0.3, 0.3, 0.3, avgPosition(100, 55, 66, 87), "move to pole");
        sleep(1000);
        score.goToPosition(1800, 0.4);
        sleep(1000);
        score.setClawPosition(0.9);
        sleep(1000);
        drive.goToPosition(-0.3, -0.3, -0.3, -0.3, avgPosition(-100, -97, -111, -98), "move back from pole");
        // lowers arm after scoring first cone
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
        // go left
        drive.goToPosition(0.3, -0.3, -0.3, 0.3, avgPosition(750,-750,-750,750), "strafe right");





        /*
        //2 middle
        drive.goToPosition(0.3, -0.3, -0.3, 0.3, avgPosition(1267, -1251, -1246, 304), "strafe right");
        //3 far right
        drive.goToPosition(0.3, -0.3, -0.3, 0.3, avgPosition(1152, -1177, -1164, 1196), "strafe right");

    */
    }
    public void blueLeft(){
        //close claw
        score.setClawPosition(0.45);
        sleep(1000);
        //lift claw a little bit
        score.goToPosition(100, 0.7);
        sleep(200);
        // move forward a square
        drive.goToPosition(0.3, 0.3,  0.3, 0.3, avgPosition(1235, 1198, 1204, 1144), "forward");
        //strafe right
        drive.goToPosition(0.3, -0.3, -0.3, 0.3, avgPosition(1448, -1398, -1598, 1500), "strafe right");
        // turn
        drive.goToPosition(0.3, -0.3, 0.3, -0.3, avgPosition(311, -325, 345, -333), "turn to pole");
        // move arm max
        score.goToPosition(2350, 0.7);
        drive.goToPosition(0.3, 0.3, 0.3, 0.3, avgPosition(326, 300, 330, 304), "move to pole");
        score.setClawPosition(0.9);
        drive.goToPosition(-0.3, -0.3, -0.3, -0.3, avgPosition(-326, -300, -330, -304), "move back from pole");
        // lowers arm after scoring first cone
        score.goToPosition(50, 0.3);
        sleep(50);

        //1 (far right) (general code)
        drive.goToPosition(-0.3, -0.3, -0.3, -0.3, avgPosition(-498, -506, -557, -565), "move back further from pole");
        sleep(0);
        //turn left a little (straighten out)
        drive.goToPosition(-0.3, 0.3, -0.3, 0.3, avgPosition(-271, 280, -260, 290), "turn straight");
        sleep(50);
        //drive forward a little
        drive.goToPosition(0.3,0.3,0.3,0.3,avgPosition(310, 380, 320, 290), "drive forward a little");



        /*
        //2
        drive.goToPosition(-0.3, 0.3, 0.3, -0.3, avgPosition(-1267, 1251, 1246, -304), "strafe left");
        //3
        drive.goToPosition(-0.3, 0.3, 0.3, -0.3, avgPosition(-1152, 1177, 1164, -1196), "strafe left");

    */
    }
    public void redRight(){

    }
    public void redLeft(){
        //close claw
        score.setClawPosition(0.45);
        sleep(1000);
        //lift claw a little bit
        score.goToPosition(100, 0.7);
        sleep(200);
        // move forward a square
        drive.goToPosition(0.3, 0.3,  0.3, 0.3, avgPosition(1235, 1198, 1204, 1144), "forward");
        //strafe right
        drive.goToPosition(0.3, -0.3, -0.3, 0.3, avgPosition(1448, -1398, -1598, 1500), "strafe right");
        // turn
        drive.goToPosition(0.3, -0.3, 0.3, -0.3, avgPosition(300, -310, 280, -322), "turn to pole");
        // move arm max
        score.goToPosition(2400, 1);
        drive.goToPosition(0.3, 0.3, 0.3, 0.3, avgPosition(326, 300, 330, 304), "move to pole");
        score.setClawPosition(0.9);
        drive.goToPosition(-0.3, -0.3, -0.3, -0.3, avgPosition(-326, -300, -330, -304), "move back from pole");
        // lowers arm after scoring first cone
        score.goToPosition(0, 0.3);
        sleep(50);

        //1 (far right) (general code)
        drive.goToPosition(-0.3, -0.3, -0.3, -0.3, avgPosition(-498, -506, -557, -565), "move back further from pole");
        sleep(50);
        //turn left a little (straighten out)
        drive.goToPosition(-0.3, 0.3, -0.3, 0.3, avgPosition(-271, 280, -260, 290), "turn straight");
        sleep(50);
        //drive forward a little
        drive.goToPosition(0.3,0.3,0.3,0.3, avgPosition(310, 380, 320, 290), "drive forward a little");



        /*
        //2
        drive.goToPosition(-0.3, 0.3, 0.3, -0.3, avgPosition(-1267, 1251, 1246, -304), "strafe left");
        //3
        drive.goToPosition(-0.3, 0.3, 0.3, -0.3, avgPosition(-1152, 1177, 1164, -1196), "strafe left");

    */
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

