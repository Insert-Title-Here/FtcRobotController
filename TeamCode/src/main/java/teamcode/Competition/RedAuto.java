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
import teamcode.common.Constants;
import teamcode.common.Localizer;
import teamcode.common.Utils;
import teamcode.common.Vector2D;
import teamcode.common.WestCoastDriveTrain;

@Autonomous(name="RedAutoFreight")
public class RedAuto extends AbstractOpMode {

    /**
     * Left is low
     * center is mid
     * right is high
     */

    WestCoastDriveTrain driveTrain;
    ArmSystem arm;
    EndgameSystems system; //carousel
    Localizer localizer;

    OpenCvWebcam webcam;
    BarcodePipeline.BarcodePosition position;

    @Override
    protected void onInitialize() {
        localizer = new Localizer(hardwareMap, new Vector2D(0,0), 0,10);
        driveTrain = new WestCoastDriveTrain(hardwareMap, localizer);
        system = new EndgameSystems(hardwareMap, true);
        arm = new ArmSystem(hardwareMap, false);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        WebcamName wc = hardwareMap.get(WebcamName.class, "Webcam");

        // W/ or W/ out live preview
        webcam = OpenCvCameraFactory.getInstance().createWebcam(wc, cameraMonitorViewId);
        // camera = OpenCvCameraFactory.getInstance().createWebcam(wc);

        BarcodePipeline pipeline = new BarcodePipeline();
        webcam.setPipeline(pipeline);
        pipeline.setSide(BarcodePipeline.Side.RED);

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
            //telemetry.addData("", position);
            //telemetry.update();
        }
    }

    @Override
    protected void onStart() {
        webcam.stopStreaming();
        telemetry.clear();
        localizer.start();
        driveTrain.moveToPosition(new Vector2D(0, 6), 12, 0.5, true);
        // Utils.sleep(2000);
        driveTrain.rotateDistance(0.5, Math.toRadians(120));


        if (position == BarcodePipeline.BarcodePosition.LEFT) {
            if(position == BarcodePipeline.BarcodePosition.LEFT){
                arm.raise(Constants.BOTTOM_POSITION - 1000);
            }
            driveTrain.moveToPosition(new Vector2D(-11.5, 19.5), -12, 0.5, false);
            driveTrain.rotateDistance(0.4, Math.toRadians(155));

        } else{
            //Debug.log("here");
            if (position == BarcodePipeline.BarcodePosition.RIGHT) {
                arm.raise(Constants.TOP_POSITION);
            } else if(position == BarcodePipeline.BarcodePosition.CENTER){
                arm.raise(Constants.MEDIUM_POSITION);
            }
            driveTrain.moveToPosition(new Vector2D(-12.5, 20.5), -12, 0.5, false);
            driveTrain.rotateDistance(0.4, Math.toRadians(155));

        }

        arm.score();
        //arm.raise(arm.getLinearSlidePosition() + 1000);
        Utils.sleep(1000);
        Vector2D constructedVector = new Vector2D(-8, 14);
//        telemetry.addData("vec", constructedVector);
//        telemetry.update();
        driveTrain.moveToPosition(constructedVector, 12, 0.5, false);
        arm.retract();
        driveTrain.rotateDistance(-0.4, Math.toRadians(90));
        driveTrain.moveToPosition(new Vector2D(34, 27), 24, 0.5, false);


        //driveTrain.moveToPosition(new Vector2D(-14,6), 12, 0.1);


        while(opModeIsActive());


//        driveTrain.moveToPosition(new Vector2D(0, 6), 12, 0.5, true);
//        // Utils.sleep(2000);
//        driveTrain.rotateDistance(0.5, Math.toRadians(120));
//        if(position == BarcodePipeline.BarcodePosition.RIGHT) {
//            arm.raise(Constants.TOP_POSITION);
//        }else{
//            arm.raise(Constants.MEDIUM_POSITION);
//
//        }
//        driveTrain.moveToPosition(new Vector2D(-7.53, 17.25), -12, 0.5, false);
//        driveTrain.rotateDistance(0.4, Math.toRadians(150));
//        arm.score();
//        Utils.sleep(500);
//        Vector2D position = localizer.getCurrentState().getPosition();
//        double rotation = localizer.getCurrentState().getRotation();
//        driveTrain.moveToPosition(new Vector2D(position.getX() - 7* Math.cos(rotation), position.getY() - 7* Math.sin(rotation)), 12, 0.5, false);
//        arm.retract();
//        driveTrain.rotateDistance(-0.4, Math.toRadians(75));
//        driveTrain.moveToPosition(new Vector2D(28, 18), 24, 0.5, false);




        //driveTrain.moveToPosition(new Vector2D(-14,6), 12, 0.1);




    }

    @Override
    protected void onStop() {
        localizer.stopThread();
    }
}
