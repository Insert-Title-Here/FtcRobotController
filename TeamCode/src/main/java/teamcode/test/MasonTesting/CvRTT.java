// TEST SCRIPT FOR OPENCV BY MASON
// link for reference: https://github.com/OpenFTC/EasyOpenCV/blob/master/doc/user_docs/camera_initialization_overview.md

package teamcode.test.MasonTesting;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

import teamcode.common.AbstractOpMode;

@TeleOp(name = "RT-Object Detection")
public class CvRTT extends AbstractOpMode {

    // Get webcam and create an OpenCvCamera
    WebcamName wc;
    OpenCvCamera camera;

    // global obj
    static final CvDetectionPipeline dp = new CvDetectionPipeline();

    @Override
    protected void onInitialize() {

        // Init webcam and create a cam object using CvFactory
        // Make sure to have the name of the webcam set in the config settings of the robot
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        wc = hardwareMap.get(WebcamName.class, "Webcam");

        // W/ or W/ out live preview
        camera = OpenCvCameraFactory.getInstance().createWebcam(wc, cameraMonitorViewId);
        // camera = OpenCvCameraFactory.getInstance().createWebcam(wc);

        camera.setPipeline(dp);

        // Open an asynchronous connection to the device
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {

            // Start opening the camera and stream it
            @Override
            public void onOpened() {

                camera.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }

            // Method will be called if the camera cannot be opened
            @Override
            public void onError(int errorCode) {
                telemetry.addData("Camera Init Error", errorCode);
            }
        });
    }

    @Override
    protected void onStart() {

        // Keep the op mode running, to keep the system from coming to a halt

        while (opModeIsActive()) {
            ArrayList<Double> xList = dp.xPointList();
            ArrayList<Double> yList = dp.yPointList();
            for (int i = 0; i < xList.size(); i++) {
                telemetry.addLine(xList.get(i) + ", " + yList.get(i));
            }
            telemetry.update();
        }
    }

    @Override
    protected void onStop() {

    }
}
