package teamcode.Competition.Autos.TankAutos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import teamcode.Competition.Subsystems.ArmSystem;
import teamcode.Competition.Subsystems.EndgameSystems;
import teamcode.Competition.Pipeline.BarcodePipeline3;
import teamcode.common.AbstractOpMode;
import teamcode.common.Constants;
import teamcode.common.Localizer;
import teamcode.common.PositionStuff.Pose;
import teamcode.common.Utils;
import teamcode.common.Vector2D;
import teamcode.common.WestCoastDriveTrain;

@Disabled
@Autonomous(name="RedAutoFreight TANK")
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
    BarcodePipeline3.BarcodePosition position;

    Thread armIntakeThread;
    private volatile boolean intake;
    private boolean eStop;

    @Override
    protected void onInitialize() {
        localizer = new Localizer(new Pose(0,0,0), hardwareMap);
        driveTrain = new WestCoastDriveTrain(hardwareMap, localizer);
        system = new EndgameSystems(hardwareMap, true);
        arm = new ArmSystem(hardwareMap, false);

        intake = false;
        armIntakeThread = new Thread(){
            @Override
            public void run(){
                for(int i = 0; i < 1; i++) {
                    while (!intake){

                    }
                    boolean isStop = arm.intakeAuto(0.7);
                    driveTrain.setEnvironmentalTerminate(true);
                    if(isStop){
                        driveTrain.seteStop(true);
                    }
                    intake = false;
                }
            }
        };

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
//        WebcamName wc = hardwareMap.get(WebcamName.class, "Webcam");
//
//        // W/ or W/ out live preview
//        webcam = OpenCvCameraFactory.getInstance().createWebcam(wc, cameraMonitorViewId);
//        BarcodePipeline3 pipeline = new BarcodePipeline3();
//        pipeline.setSide(BarcodePipeline3.Side.RED);
//        webcam.setPipeline(pipeline);
//
//        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
//            @Override
//            public void onOpened() {
//                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT); //specify cam orientation and calibrate the resolution
//            }
//
//            @Override
//            public void onError(int errorCode) {
//                telemetry.addData("Camera Init Error", errorCode);
//                telemetry.update();
//            }
//        });
        localizer.lowerOdo();
        while(!opModeIsActive()){
            position = BarcodePipeline3.BarcodePosition.LEFT;
            //telemetry.addData("", position);
            //telemetry.update();
        }
    }

    @Override
    protected void onStart() {
        //webcam.stopStreaming();
        telemetry.clear();
        localizer.start();
        armIntakeThread.start();

        Utils.sleep(1500);
        driveTrain.moveToPosition(new Vector2D(-1, 6), 12, 0.5, false);
        // Utils.sleep(2000);
        driveTrain.rotateDistance(0.5, Math.toRadians(-120));

        if (position == BarcodePipeline3.BarcodePosition.RIGHT) {
            arm.raise(Constants.TOP_POSITION);
        } else if(position == BarcodePipeline3.BarcodePosition.CENTER){
            arm.raise(Constants.MEDIUM_POSITION);
        }
        driveTrain.moveToPosition(new Vector2D(-14.5, 23.5), -12, 0.5, false);
        driveTrain.rotateDistance(0.4, Math.toRadians(-170));
        if (position == BarcodePipeline3.BarcodePosition.LEFT) {
            arm.raise(Constants.BOTTOM_POSITION);
            Utils.sleep(250);
            arm.runConveyorPos(1, 2000);
        }
        arm.score();
        //driveTrain.rotateDistance( 0.4, Math.toRadians(155));

        //arm.raise(arm.getLinearSlidePosition() + 1000);
        Utils.sleep(1000);
        Vector2D constructedVector = new Vector2D(-13, 19);
//        telemetry.addData("vec", constructedVector);
//        telemetry.update();
        //driveTrain.moveToPosition(constructedVector, 12, 0.5, false);
        arm.retract();
        driveTrain.rotateDistance(-0.4, Math.toRadians(-105));
        localizer.liftOdo();
        driveTrain.moveToPosition(new Vector2D(36, -29), 36, 0.5, false);
//        driveTrain.rotateDistance(0.5,Math.toRadians(-120));
//       intake = true;
//        driveTrain.moveToPosition(new Vector2D(37, -30), 12, 0.5, false);
//        driveTrain.moveToPosition(new Vector2D(35, -32), -24, 0.5, false);

//        driveTrain.rotateDistance(-0.5,Math.toRadians(-120));
//        driveTrain.moveToPosition(new Vector2D(10,-12), -36, 0.5, false);
//        //driveTrain.rotateDistance(-0.5,Math.toRadians(-105));
//
//        arm.raise(Constants.TOP_POSITION);
//        arm.score();
//        Utils.sleep(200);
//        //driveTrain.rotateDistance(-0.4, Math.toRadians(-105));
//        driveTrain.moveToPosition(new Vector2D(34, -27), 36, 0.5, false);
//
//        arm.retract();

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
        armIntakeThread.interrupt();
    }
}
