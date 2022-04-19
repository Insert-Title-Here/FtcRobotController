package teamcode.test.MasonTesting;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import teamcode.common.AbstractOpMode;

@Disabled
@Autonomous (name = "DistanceTestV0.03")
public class CvRTT2 extends AbstractOpMode {

    WebcamName wc;
    OpenCvCamera camera;

    DistanceDetection dp = new DistanceDetection();

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

        while (opModeIsActive()) {
//            telemetry.addData(dp.getNearestDistance(), null);
//            telemetry.update();
        }
    }

    @Override
    protected void onStop() {
        camera.stopStreaming();
    }
}
