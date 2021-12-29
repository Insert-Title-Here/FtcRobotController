package teamcode.test.MasonTesting;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import teamcode.Competition.Pipeline.BarcodePipeline3;
import teamcode.Competition.Subsystems.ArmSystem;
import teamcode.Competition.Subsystems.EndgameSystems;
import teamcode.common.AbstractOpMode;
import teamcode.common.Localizer;
import teamcode.common.PositionStuff.Pose;
import teamcode.common.Utils;
import teamcode.common.WestCoastDriveTrain;

@Autonomous(name = "DetectionAutoTest")
public class DetectionAutoTest extends AbstractOpMode {
    WestCoastDriveTrain driveTrain;
    ArmSystem arm;
    EndgameSystems system; //carousel
    Localizer localizer;

    OpenCvWebcam webcam;
    BarcodePipeline3.BarcodePosition position;

    @Override
    protected void onInitialize() {
        localizer = new Localizer(new Pose(0,0,0), hardwareMap);
        driveTrain = new WestCoastDriveTrain(hardwareMap, localizer);
        arm = new ArmSystem(hardwareMap, false);
        system = new EndgameSystems(hardwareMap, true);

        localizer.lowerOdo();

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        WebcamName wc = hardwareMap.get(WebcamName.class, "Webcam");

        webcam = OpenCvCameraFactory.getInstance().createWebcam(wc, cameraMonitorViewId);

        CvDetectionPipeline detPipeline = new CvDetectionPipeline();
        webcam.setPipeline(detPipeline);

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
        while (!opModeIsActive()) {

        }
    }

    @Override
    protected void onStart() {
        webcam.stopStreaming();
        telemetry.clear();
        localizer.start();
        Utils.sleep(1500);
    }

    @Override
    protected void onStop() {
        localizer.stopThread();
    }
}
