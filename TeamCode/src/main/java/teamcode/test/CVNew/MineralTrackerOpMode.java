package teamcode.test.CVNew;

import android.graphics.Bitmap;
import android.graphics.Color;
import android.os.Environment;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.vuforia.Image;
import com.vuforia.PIXEL_FORMAT;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;

import teamcode.common.AbstractOpMode;
import teamcode.common.Vector2D;

import static android.graphics.Bitmap.createBitmap;
import static android.graphics.Bitmap.createScaledBitmap;

@TeleOp(name = "Vuforia")
public class MineralTrackerOpMode extends AbstractOpMode {
    VuforiaLocalizer vuforia = null;
    OpenCvCamera vuforiaPassthroughCam;

    @Override
    protected void onInitialize() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        int[] viewportContainerIds = OpenCvCameraFactory.getInstance().splitLayoutForMultipleViewports(cameraMonitorViewId, 2, OpenCvCameraFactory.ViewportSplitMethod.VERTICALLY);

        /*
         * Setup Vuforia
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(viewportContainerIds[0]);
        parameters.vuforiaLicenseKey = "AdmY9bL/////AAABmdI1tj5yQU6Hquyg0Z7OFtWDWBWTbdy/9MbWJVG5oWd9UOD2XbUKef6bVMWmV+UDC6LX3Z4Vzvnlre6PvT+oP4g0NHyslh3z2bbip9Qz0v48ENmQqF8bJXzVE6mRoPXSfeRQQGjP8T5DW1+C9sDs8JOtLmlPL0rsUfe4BUwaZ7/D8JzwCciw6JNtNqmesL4n3S/yCNk6+M1aDsDLNmzpHHeAb8BzuLTjBRLJIYRLKywebaEZg9IykdDkgzlr/YRLGDqkuu8zd9P2lF4cycP6dWhbGNk7a6dQ4RfGoRWMgI0PWI/UPzlDHFYwHwtXkkAtqFKbfR5zAW8hGmcYzoF+N6mNzd/Ebq6VJxjbt4d9mt1d";
        parameters.cameraDirection   = VuforiaLocalizer.CameraDirection.BACK;
        // Uncomment this line below to use a webcam
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Create a Vuforia passthrough "virtual camera"
        vuforiaPassthroughCam = OpenCvCameraFactory.getInstance().createVuforiaPassthrough(vuforia, parameters, viewportContainerIds[1]);

        vuforiaPassthroughCam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                // Using GPU acceleration can be particularly helpful when using Vuforia passthrough
                // mode, because Vuforia often chooses high resolutions (such as 720p) which can be
                // very CPU-taxing to rotate in software. GPU acceleration has been observed to cause
                // issues on some devices, though, so if you experience issues you may wish to disable it.
                vuforiaPassthroughCam.setViewportRenderer(OpenCvCamera.ViewportRenderer.GPU_ACCELERATED);
                vuforiaPassthroughCam.setViewportRenderingPolicy(OpenCvCamera.ViewportRenderingPolicy.OPTIMIZE_VIEW);


                // We don't get to choose resolution, unfortunately. The width and height parameters
                // are entirely ignored when using Vuforia passthrough mode. However, they are left
                // in the method signature to provide interface compatibility with the other types
                // of cameras.
                vuforiaPassthroughCam.startStreaming(0,0, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {
                /*
                 * This will be called if the camera could not be opened
                 */
            }
        });

    }

    @Override
    protected void onStart() {

    }

    private static final String VUFORIA_KEY = "AQR2KKb/////AAABmcBOjjqXfkjtrjI9/Ps5Rs1yoVMyJe0wdjaX8pHqOaPu2gRcObwPjsuWCCo7Xt52/kJ4dAZfUM5Gy73z3ogM2E2qzyVObda1EFHZuUrrYkJzKM3AhY8vUz6R3fH0c/R9j/pufFYAABOAFoc5PtjMQ2fbeFI95UYXtl0u+6OIkCUJ3Zw71tvoD9Fs/cOiLB45FrWrxHPbinEhsOlCTWK/sAC2OK2HuEsBFCebaV57vKyATHW4w2LMWEZaCByHMk9RJDR38WCqivXz753bsiBVMbCzPYzwzc3DKztTbK8/cXqPPBLBKwU8ls0RN52akror1xE9lPwwksMXwJwolpyIZGnZngWcBWX4lLH+HlDNZ8Qm";

    private static final String ASSET_NAME = "stones";


    private VuforiaTrackables trackables;




    private VuforiaLocalizer createVuforia(HardwareMap hardwareMap, CameraType cameraType) {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().
                getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        if (cameraType == CameraType.WEBCAM) {
            parameters.cameraName = hardwareMap.get(CameraName.class,"webcam");
        }
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        VuforiaLocalizer vuforia = ClassFactory.getInstance().createVuforia(parameters);
        trackables = vuforia.loadTrackablesFromAsset(ASSET_NAME);
        trackables.activate();
        return vuforia;
    }

    public Vector2D getBallPosition() {
        VuforiaTrackable ball = trackables.get(0);
        OpenGLMatrix pose = ((VuforiaTrackableDefaultListener) ball.getListener()).getPose();
        if (pose == null) {
            return null;
        }
        VectorF position = pose.getTranslation();
        double x = position.get(0);
        double y = position.get(1);
        return new Vector2D(x, y);
    }

    public enum CameraType {
        PHONE, WEBCAM
    }




    @Override
    protected void onStop() {

    }
}
