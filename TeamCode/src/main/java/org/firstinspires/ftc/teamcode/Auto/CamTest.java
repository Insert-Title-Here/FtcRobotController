package org.firstinspires.ftc.teamcode.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

@Autonomous
public class CamTest extends LinearOpMode {
    DetectionAlgorithm detect;
    String position; //temp

    OpenCvWebcam webcam;
    int parkLocation;
    @Override
    public void runOpMode() throws InterruptedException {
        detect = new DetectionAlgorithm();
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        webcam.setPipeline(detect);

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {

            @Override
            public void onOpened() {
                webcam.startStreaming(320, 176, OpenCvCameraRotation.SIDEWAYS_LEFT);
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addData("Camera Init Error", errorCode);
                telemetry.update();
            }
        });

        telemetry.addData("position", position);
        telemetry.addData("Status", "Initialized");

        telemetry.update();



        waitForStart();
        webcam.stopStreaming();

    }

    //TODO: check if camera angle works
    private class DetectionAlgorithm extends OpenCvPipeline {
        Mat original;
        Mat changed;


        @Override
        public Mat processFrame(Mat input) {
            original = new Mat();
            changed = new Mat();

            input.copyTo(original);
            changed = new Mat();
            if(original.empty()) {
                return input;
            }
            // cyan magenta yellow
            Core.extractChannel(original, changed, 1);
            //Imgproc.cvtColor(original, changed, Imgproc.COLOR_RGB2YCrCb);
            Imgproc.GaussianBlur(changed, changed, new Size(3,3), 0);
            Imgproc.dilate(changed, changed, new Mat(), new Point(-1, -1), 3);
            Imgproc.erode(changed, changed, new Mat(), new Point(-1, -1), 3);

            /* submatrices
            Mat pixel_section = original.submat(rowStart, rowEnd, colStart, colEnd)l

             */



            // magenta 255, 0, 255
            //Core.inRange(changed, new Scalar(240, 0 ,240), new Scalar(255, 0, 255), changed);
            return changed;
        }
    }


}

