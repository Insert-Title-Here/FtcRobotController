package teamcode.test.MasonTesting;


import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Rect2d;
import org.opencv.video.TrackerMIL;
import org.opencv.video.TrackerMIL_Params;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvTracker;
import org.openftc.easyopencv.OpenCvTrackerApiPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

import teamcode.common.AbstractOpMode;

public class TrackerTest extends AbstractOpMode {

    private Rect BOUNDING_BOX = new Rect(0,0,0,0);//this is not correct this would need to be calculated
    OpenCvTrackerApiPipeline pipeline;
     TrackerMIL tracker;
     TrackerMIL_Params params;
     OpenCvTracker milWrapper;
    private WebcamName wc;
    private OpenCvWebcam camera;


    @Override
    protected void onInitialize() {
        milWrapper = new OpenCvTracker() {
            @Override
            public Mat processFrame(Mat input) {
                tracker.update(input, BOUNDING_BOX);
                return input;
            }
        };
        params = new TrackerMIL_Params();
        tracker = TrackerMIL.create(params);
        pipeline.addTracker(milWrapper);
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        wc = hardwareMap.get(WebcamName.class, "Webcam");

        // W/ or W/ out live preview
        camera = OpenCvCameraFactory.getInstance().createWebcam(wc, cameraMonitorViewId);
        // camera = OpenCvCameraFactory.getInstance().createWebcam(wc);

        camera.setPipeline(pipeline);

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
        while(opModeIsActive()){

        }
    }

    @Override
    protected void onStop() {

    }
}
