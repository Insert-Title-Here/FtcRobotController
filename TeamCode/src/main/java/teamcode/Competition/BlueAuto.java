package teamcode.Competition;

import com.intel.realsense.librealsense.Pipeline;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import teamcode.Competition.BarcodePipeline;
import teamcode.common.AbstractOpMode;
import teamcode.common.Localizer;
import teamcode.common.Utils;
import teamcode.common.Vector2D;
import teamcode.common.WestCoastDriveTrain;

@Autonomous(name="BlueAuto")
public class BlueAuto extends AbstractOpMode {

    WestCoastDriveTrain driveTrain;
    ArmSystem arm;
    EndgameSystems system; //carousel
    Localizer localizer;

    OpenCvWebcam webcam;
    BarcodePipeline.BarcodePosition position;

    @Override
    protected void onInitialize() {
        localizer = new Localizer(hardwareMap, new Vector2D(34,9), 0,10);
        driveTrain = new WestCoastDriveTrain(hardwareMap, localizer);
        system = new EndgameSystems(hardwareMap, true);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        WebcamName wc = hardwareMap.get(WebcamName.class, "Webcam");

        // W/ or W/ out live preview
        webcam = OpenCvCameraFactory.getInstance().createWebcam(wc, cameraMonitorViewId);
        // camera = OpenCvCameraFactory.getInstance().createWebcam(wc);

        BarcodePipeline pipeline = new BarcodePipeline();
        webcam.setPipeline(pipeline);

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT); //specify cam orientation and calibrate the resolution
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addData("Camera Init Error", errorCode);
                telemetry.update();
            }
        });
        while(!opModeIsActive()){
            position = pipeline.getPos();
            telemetry.addData("", position);
            telemetry.update();
        }
    }

    @Override
    protected void onStart() {
        telemetry.clear();
        localizer.start();
        driveTrain.moveToPosition(new Vector2D(34, 15), 12, 0.5);
       // Utils.sleep(2000);
        driveTrain.rotateDistance(0.5, Math.PI / 2.0);
        driveTrain.moveToPosition(new Vector2D(20,15), 12, 0.1);


        while(opModeIsActive());


    }

    @Override
    protected void onStop() {
        localizer.stopThread();
    }
}
