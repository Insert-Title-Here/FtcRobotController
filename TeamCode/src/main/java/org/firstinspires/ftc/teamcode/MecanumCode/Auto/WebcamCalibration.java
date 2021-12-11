package org.firstinspires.ftc.teamcode.MecanumCode.Auto;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.MecanumCode.Common.OpModeWrapper;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.io.FileNotFoundException;

@TeleOp(name = "Webcam Calibration")
public class WebcamCalibration extends OpModeWrapper {

    WebcamName webcam;
    OpenCvCamera camera;

    // CHANGE TO RED OR BLUE DEPENDING ON THE SIDE BEING CALIBRATED/TESTED
    static BarcodePipeline.AutoSide side = BarcodePipeline.AutoSide.RED;

    static final BarcodePipeline bcPipeline = new BarcodePipeline(side);

    @Override
    protected void onInitialize() throws FileNotFoundException {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = hardwareMap.get(WebcamName.class, "Webcam");

        camera = OpenCvCameraFactory.getInstance().createWebcam(webcam, cameraMonitorViewId);
        camera.setPipeline(bcPipeline);

        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addData("Camera Init Error", errorCode);
            }
        });
    }

    @Override
    protected void onStart() {
        while (opModeIsActive()) {
            telemetry.addData("left: ", bcPipeline.avg1);
            telemetry.addData("middle: ", bcPipeline.avg2);
            telemetry.addData("pos:", bcPipeline.getPos());
            telemetry.update();
        }
    }

    @Override
    protected void onStop() {

    }
}
