package teamcode.test.MasonTesting;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvWebcam;

import teamcode.common.AbstractOpMode;
import teamcode.common.Localizer;
import teamcode.common.MecanumDriveTrain;
import teamcode.common.PositionStuff.Pose;
import teamcode.common.WestCoastDriveTrain;

public class RTTImplTest extends AbstractOpMode {
    Localizer localizer;
    MecanumDriveTrain drive;
    OpenCvWebcam webcam;
    CvDetectionPipeline pipeline;


    @Override
    protected void onInitialize() {
        localizer = new Localizer(new Pose(0,0,0), hardwareMap);
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        WebcamName wc = hardwareMap.get(WebcamName.class, "Webcam");

        // W/ or W/ out live preview
        webcam = OpenCvCameraFactory.getInstance().createWebcam(wc, cameraMonitorViewId);
        webcam.setPipeline(pipeline);
        drive = new MecanumDriveTrain(hardwareMap, localizer, pipeline, false);
    }

    @Override
    protected void onStart() {
        webcam.startStreaming(320,240);
        localizer.start();
        drive.seekCubes(12, 0.5);

    }

    @Override
    protected void onStop() {

    }
}
