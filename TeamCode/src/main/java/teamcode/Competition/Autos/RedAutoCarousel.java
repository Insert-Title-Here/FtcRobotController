package teamcode.Competition.Autos;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import teamcode.Competition.Pipeline.CarouselPipeline;
import teamcode.Competition.Subsystems.ArmSystem;
import teamcode.Competition.Subsystems.EndgameSystems;
import teamcode.Competition.Pipeline.BarcodePipeline3;
import teamcode.common.AbstractOpMode;
import teamcode.common.Constants;
import teamcode.common.Debug;
import teamcode.common.Localizer;
import teamcode.common.PositionStuff.Pose;
import teamcode.common.Utils;
import teamcode.common.Vector2D;
import teamcode.common.WestCoastDriveTrain;

@Autonomous(name="CarouselRed")
public class RedAutoCarousel extends AbstractOpMode {
    WestCoastDriveTrain driveTrain;
    ArmSystem arm;
    EndgameSystems system; //carousel
    Localizer localizer;

    OpenCvWebcam webcam;
    CarouselPipeline.BarcodePosition position;

    @Override
    protected void onInitialize() {
        localizer = new Localizer(new Pose(0,0,0), hardwareMap);
        driveTrain = new WestCoastDriveTrain(hardwareMap, localizer);
        system = new EndgameSystems(hardwareMap, true);
        arm = new ArmSystem(hardwareMap, false);

        localizer.lowerOdo();

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        WebcamName wc = hardwareMap.get(WebcamName.class, "Webcam");

        // W/ or W/ out live preview
        webcam = OpenCvCameraFactory.getInstance().createWebcam(wc, cameraMonitorViewId);
        // camera = OpenCvCameraFactory.getInstance().createWebcam(wc);

        CarouselPipeline pipeline = new CarouselPipeline();
        pipeline.setSide(CarouselPipeline.Side.RED);
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
            //telemetry.addData("", position);
            //telemetry.update();
        }
    }

    @Override
    protected void onStart() {
        webcam.stopStreaming();
        telemetry.clear();
        localizer.start();
        driveTrain.moveToPosition(new Vector2D(6, 1), 12, 0.5, false);
        driveTrain.rotateDistance(-0.5, Math.toRadians(120));

        if (position == CarouselPipeline.BarcodePosition.RIGHT) {
            Debug.log("top");
            arm.raise(Constants.TOP_POSITION );
        } else if(position == CarouselPipeline.BarcodePosition.CENTER){
            Debug.log("mid");
            arm.raise(Constants.MEDIUM_POSITION);
        }
        driveTrain.moveToPosition(new Vector2D(-19, 24), -12, 0.5, false);
        driveTrain.rotateDistance(-0.4, Math.toRadians(-155));
        if (position == CarouselPipeline.BarcodePosition.LEFT) {
           Debug.log("low");
            arm.raise(Constants.BOTTOM_POSITION - 1000);
            arm.runConveyorPos(1, 2000);
        }
        arm.score();
        Utils.sleep(500) ;
        driveTrain.rotateDistance(0.4, Math.toRadians(105));
        driveTrain.moveToPosition(new Vector2D(-0, -10), 12, 0.5, false);
        arm.retract();


        // Utils.sleep(2000);



        while(opModeIsActive());


    }

    @Override
    protected void onStop() {
        localizer.stopThread();
    }

    /*
    driveTrain.rotateDistance(0.5, Math.toRadians(75));

        driveTrain.moveToPosition(new Vector2D(19,6), 12, 0.1, false);
        system.scoreDuckAuto();
        driveTrain.moveToPosition(new Vector2D(0, 6), -12, 0.5, false);
        driveTrain.rotateDistance( 0.5, Math.toRadians(135));


        if (position == BarcodePipeline3.BarcodePosition.LEFT) {
            if(position == BarcodePipeline3.BarcodePosition.LEFT){
                arm.raise(Constants.BOTTOM_POSITION - 1000);
            }
            driveTrain.moveToPosition(new Vector2D(-12.8, 20.8), -12, 0.5, false);
            driveTrain.rotateDistance(0.4, Math.toRadians(155));

        } else{
            //Debug.log("here");
            if (position == BarcodePipeline3.BarcodePosition.RIGHT) {
                arm.raise(Constants.TOP_POSITION);
            } else if(position == BarcodePipeline3.BarcodePosition.CENTER){
                arm.raise(Constants.MEDIUM_POSITION);
            }
            driveTrain.moveToPosition(new Vector2D(-14.2, 22.2), -12, 0.5, false);
            //driveTrain.rotateDistance(-0.4, Math.toRadians(130));

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
        driveTrain.moveToPosition(new Vector2D(31,19), 12, 0.1, false);

     */
}
