package teamcode.Competition.Autos.TankAutos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import teamcode.Competition.Pipeline.BarcodePipeline3;
import teamcode.Competition.Subsystems.ArmSystem;
import teamcode.Competition.Subsystems.EndgameSystems;
import teamcode.Competition.Pipeline.BarcodePipeline;
import teamcode.common.AbstractOpMode;
import teamcode.common.Constants;
import teamcode.common.Localizer;
import teamcode.common.PositionStuff.Pose;
import teamcode.common.Utils;
import teamcode.common.Vector2D;
import teamcode.common.WestCoastDriveTrain;

@Disabled
@Autonomous(name="BlueAutoFreight")
public class BlueAuto extends AbstractOpMode {

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

        // W/ or W/ out live preview
        webcam = OpenCvCameraFactory.getInstance().createWebcam(wc, cameraMonitorViewId);
        // camera = OpenCvCameraFactory.getInstance().createWebcam(wc);

        BarcodePipeline3 pipeline = new BarcodePipeline3();
        webcam.setPipeline(pipeline);
        pipeline.setSide(BarcodePipeline3.Side.BLUE);

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
        webcam.stopStreaming();
        telemetry.clear();
        localizer.start();
        Utils.sleep(1500);
        driveTrain.moveToPosition(new Vector2D(-1, 6), 12, 0.5, false);
        // Utils.sleep(2000);
        driveTrain.rotateDistance(-0.5, Math.toRadians(120));

        if (position == BarcodePipeline3.BarcodePosition.LEFT) {
            arm.raise(Constants.TOP_POSITION);
        } else if(position == BarcodePipeline3.BarcodePosition.CENTER){
            arm.raise(Constants.MEDIUM_POSITION);
        }
        driveTrain.moveToPosition(new Vector2D(-16, 23.5), -12, 0.5, false);
        driveTrain.rotateDistance(-0.4, Math.toRadians(170));
        if (position == BarcodePipeline3.BarcodePosition.RIGHT) {
            arm.raise(Constants.BOTTOM_POSITION);
            arm.runConveyorPos(1, 1500);
        }
        arm.score();
        //driveTrain.rotateDistance( 0.4, Math.toRadians(155));

        //arm.raise(arm.getLinearSlidePosition() + 1000);
        Utils.sleep(1000);
        Vector2D constructedVector = new Vector2D(-13, 19);
//        telemetry.addData("vec", constructedVector);
//        telemetry.update();
        //driveTrain.moveToPosition(constructedVector, 12, 0.5, false);
        driveTrain.rotateDistance(0.4, Math.toRadians(105));
        arm.retract();
        localizer.liftOdo();
        driveTrain.moveToPosition(new Vector2D(-36, -29), 36, 0.5, false);
        while(opModeIsActive());


    }

    @Override
    protected void onStop() {
        localizer.stopThread();
    }
}
