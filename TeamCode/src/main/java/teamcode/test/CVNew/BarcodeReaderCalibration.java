package teamcode.test.CVNew;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvWebcam;

import teamcode.Competition.BarcodeReaderPipeline;
import teamcode.common.AbstractOpMode;

@Autonomous(name="BarcodeCalibrator")
public class BarcodeReaderCalibration extends AbstractOpMode {

    BarcodeReaderPipeline pipeline;
    OpenCvWebcam webcam;
    BarcodeReaderPipeline.BarcodePosition position;
    @Override
    protected void onInitialize() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam"), cameraMonitorViewId);
        pipeline = new BarcodeReaderPipeline();
        webcam.setPipeline(pipeline);

    }

    @Override
    protected void onStart() {
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(320, 240);
            }

            @Override
            public void onError(int errorCode) {

            }
        });

        while(opModeIsActive()){
            position = pipeline.getBarcodePosition();
            telemetry.addData("position", position);
            telemetry.update();
        }

    }

    @Override
    protected void onStop() {

    }
}
