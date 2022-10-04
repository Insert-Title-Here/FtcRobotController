package org.firstinspires.ftc.teamcode.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Common.MecanumDrive;
import org.firstinspires.ftc.teamcode.Common.ScoringSystem;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

@Autonomous
public class MecDriveTest extends LinearOpMode {
    MecanumDrive drive;
    ScoringSystem score;
    DetectionAlgorithm detect;
    String position; //temp

    OpenCvWebcam webcam;
    int parkLocation;
    @Override
    public void runOpMode() throws InterruptedException {
        drive = new MecanumDrive(hardwareMap, telemetry);
        score = new ScoringSystem(hardwareMap);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "webcam"), cameraMonitorViewId);
        webcam.setPipeline(detect);
        webcam.startStreaming(320, 176);
        telemetry.addData("position", position);
        telemetry.addData("Status", "Initialized");

        telemetry.update();


        // TODO: Continued... this awas a linear OPMODE; wmake sure you are calling opModelIsActive() in any loops.
        /*
        -put cone on pole, get new cone, repeat
        -read custom sleeve, remember place to park accordingly
         */

        //Example Code for Direction
        /*
        //forward
        drive.goToPosition(0.3, 0.3, 0.3, 0.3, 5000);

        //backward
        drive.goToPosition(-0.3, -0.3, -0.3, -0.3, 5000);

        //Left
        drive.goToPosition(0.3, -0.3, 0.3, -0.3, 5000);

        //Right
        drive.goToPosition(-0.3, 0.3, 0.3, -0.3, 5000);

        //Forward-Left Diagonal
        drive.goToPosition(0.3, 0, 0, 0.3, 3000);

        //Forward-Right Diagonal
        drive.goToPosition(0, 0.3, 0.3, 0, 3000);

        //Backward-Left Diagonal
        drive.goToPosition(-0.3, 0, 0, -0.3, 3000);

        //Backward-Right Diagonal
        drive.goToPosition(0, -0.3, -0.3, 0, 3000);
         */

        // Camera checks sleeve...stores parking location??
        score.setClawPosition(0.9);


        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();
        webcam.stopStreaming();
        //close claw
        score.setClawPosition(0.48);


        //lift claw a little bit
        //score.goToPosition(300, 0.3);
        drive.goToPosition(-0.1, -0.1, 0.1, 0.1, 1000, "forward");
        /*
        // move forward a square
        drive.goToPosition(-0.3, -0.3, 0.3, 0.3, avgPosition(-1235, -1198, 1204, 1144)/4, "forward");
        sleep(50);
        //strafe left
        drive.goToPosition(-0.3, 0.3, -0.3, 0.3, avgPosition(1448, -1398, 1598, -1500), "strafe right");
        sleep(50);
        // turn -> 406, -397, -438, 440
        // move arm max
        score.goToPosition(2390, 1);

        drive.goToPosition(-0.3, -0.3, 0.3, 0.3, avgPosition(-326, -300, 330, 304), "move to pole");
        score.setClawPosition(0.9);
        score.goToPosition(0, 0.5);
        sleep(50);

        //1 (far left) (general code)
        drive.goToPosition(0.3, 0.3, -0.3, -0.3, avgPosition(498, 506, -557, -565), "move back from pole");
        sleep(50);
        //turn -> -395, 368, 413, -410
        drive.goToPosition(-0.3, -0.3, 0.3, 0.3, avgPosition(-98, -95, 105, 92), "move forward a bit");
        sleep(50);

         */

        /*
        //2
        drive.goToPosition(-0.3, 0.3, -0.3, 0.3, avgPosition(-1267, 1251, -1246, 304), "strafe right");
        sleep(50);
        //3
        drive.goToPosition(-0.3, 0.3, -0.3, 0.3, avgPosition(-1152, 1177, -1164, 1196), "strafe right");

        sleep(50);
    */






    }
    public int avgPosition(int fl, int fr, int bl, int br){
        return (int)(Math.abs(fl) + Math.abs(fr) + Math.abs(bl) + Math.abs(br))/4;
    }

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
            Imgproc.GaussianBlur(changed, changed, new Size(2,2), 0);
            /* submatrices
            Mat pixel_section = original.submat(rowStart, rowEnd, colStart, colEnd)l

             */



            // magenta 255, 0, 255
            Core.inRange(changed, new Scalar(240, 0 ,240), new Scalar(255, 0, 255), changed);
            return changed;
        }
    }


}

