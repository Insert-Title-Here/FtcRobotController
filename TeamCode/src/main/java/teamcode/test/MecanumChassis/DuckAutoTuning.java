package teamcode.test.MecanumChassis;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import teamcode.Competition.Pipeline.MecanumPipeline.MecanumBarcodePipeline;
import teamcode.Competition.Subsystems.ArmSystem;
import teamcode.Competition.Subsystems.EndgameSystems;
import teamcode.common.AbstractOpMode;
import teamcode.common.Debug;
import teamcode.common.MecanumDriveTrain;
import teamcode.common.Utils;
import teamcode.test.MasonTesting.DuckPipeline;

@Autonomous(name="duck auto tuning ")
public class DuckAutoTuning extends AbstractOpMode {

    MecanumDriveTrain drive;
    ArmSystem system;
    EndgameSystems systems;
    DuckPipeline duck;
    private OpenCvWebcam webcam;
    @Override
    protected void onInitialize() {
        duck = new DuckPipeline();
        system = new ArmSystem(hardwareMap, false);
        systems = new EndgameSystems(hardwareMap, false);
        drive = new MecanumDriveTrain(hardwareMap, true, systems, system);
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        WebcamName wc = hardwareMap.get(WebcamName.class, "Webcam");

        // W/ or W/ out live preview
        webcam = OpenCvCameraFactory.getInstance().createWebcam(wc, cameraMonitorViewId);
        webcam.setPipeline(duck);
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
    }

    private double VELOCITY = 10;
    @Override
    protected void onStart() {
        drive.strafeDistanceSensor(6, 0);
        system.setIsDuck(true);
        Utils.sleep(200);
        //drive.moveDistanceDEVelocity(300, 180, VELOCITY);

        drive.moveDistanceDEVelocity(350, -90, VELOCITY);
        Utils.sleep(200);

        drive.moveDistanceDEVelocity(600, 0, VELOCITY / 2.0);//
        drive.driveColorSensorWarehouse(4);
        drive.duck();
        system.lowerLinkage();
        system.intakeDumb(1.0);

        systems.scoreDuckAuto();
        Utils.sleep(1000);
        drive.moveDistanceDEVelocity(400, 180, VELOCITY / 2.0);
        Utils.sleep(500);

        NormalizedRGBA rgba = drive.getSensorRGBA();
            double direction = duck.direction();
            while (!duck.isCentered()) {
                telemetry.addData("DIR: ", duck.direction());
                telemetry.addData("is", duck.isCentered());
                telemetry.update();
                direction = duck.direction();
                drive.setStrafe(direction * 0.5);
            }

            drive.brake();
            Utils.sleep(2000);
            //drive.moveDistanceDEVelocity(100, -90, VELOCITY / 2.0);

            drive.driveColorSensorNoWarehouse(1);
            telemetry.clear();
            Debug.log("intook");

           drive.moveDistanceDEVelocity(400, 180, VELOCITY);
        }

    @Override
    protected void onStop() {

    }


}
