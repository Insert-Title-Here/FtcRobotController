package teamcode.Competition.Autos.MecanumAutos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import teamcode.Competition.Pipeline.TankPipeline.BarcodePipeline3;
import teamcode.Competition.Subsystems.ArmSystem;
import teamcode.Competition.Subsystems.EndgameSystems;
import teamcode.common.AbstractOpMode;
import teamcode.common.Constants;
import teamcode.common.Debug;
import teamcode.common.Localizer;
import teamcode.common.MecanumDriveTrain;
import teamcode.common.PositionStuff.Pose;
import teamcode.common.Utils;
import teamcode.common.Vector2D;

@Autonomous(name="CarouselRed")
public class CarouselAutoRed extends AbstractOpMode {

    MecanumDriveTrain drive;
    ArmSystem arm;
    Localizer localizer;
    EndgameSystems system;

    OpenCvWebcam webcam;
    BarcodePipeline3.BarcodePosition position;

    Thread secondaryFunctionsThread, timerThread;

    private final double VELOCITY = 15;
    private final double OMEGA = 0.4;

    double deltaTime, initialTime;
    volatile currentCycleState state;
    volatile boolean executeArmCommands;


    @Override
    protected void onInitialize() {
        localizer = new Localizer(new Pose(0,0,0), hardwareMap);
        drive = new MecanumDriveTrain(hardwareMap, localizer, true);
        arm = new ArmSystem(hardwareMap, false);
        system = new EndgameSystems(hardwareMap, false);

        secondaryFunctionsThread = new Thread(){
            @Override
            public void run(){
                secondaryFunctionsSequence();
            }
        };

        deltaTime = 0;

        timerThread = new Thread(){
            public void run(){
                beginTiming();
            }
        };
        state = currentCycleState.PRELOAD;

//        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
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
        executeArmCommands = true;
        extend = false;
        retract = false;
        spin = false;
        duckExit = false;
        while(!opModeIsActive()){
            position = BarcodePipeline3.BarcodePosition.RIGHT;
            //telemetry.addData("", position);
            //telemetry.update();
        }
    }



    volatile boolean duckExit, spin, extend, retract;
    //contingency plan for 0 freight, we set this to fencepost or comment out the intake section of it
    private void secondaryFunctionsSequence(){
        while(!spin && opModeIsActive());
        if(executeArmCommands){
            system.scoreDuckAuto();
        }
        spin = false;
        Debug.log("here");
        duckExit = true;

        while(!retract && opModeIsActive());
        Utils.sleep(250);
        Debug.log("here");
        if(executeArmCommands) {
            arm.retract();
        }
        retract = false;

    }

    @Override
    protected void onStart() {
        localizer.start();
        secondaryFunctionsThread.start();
        telemetry.clear();
        drive.moveToPosition(new Vector2D(9,0),  VELOCITY);

        //drive.moveToPosition(new Vector2D(20, 16), VELOCITY);
        drive.rotateDistance(Math.toRadians(-90), -OMEGA);
        drive.moveToPosition(new Vector2D(14,-20), VELOCITY);
        drive.moveToPosition(new Vector2D(5.5,-20), VELOCITY);
        spin = true;
        //drive.setStrafe(-0.1);
        while(!duckExit && opModeIsActive());
        drive.moveToPosition(new Vector2D(40,-17), VELOCITY);
        drive.rotateDistance(Math.toRadians(-90), OMEGA);
        drive.moveToPosition(new Vector2D(45,5), VELOCITY);
        drive.rotateDistance(Math.toRadians(-90), -OMEGA);

        if (position == BarcodePipeline3.BarcodePosition.RIGHT) {
            arm.raise(Constants.BOTTOM_POSITION - 1500);
        } else if (position == BarcodePipeline3.BarcodePosition.CENTER) {
            arm.raise(Constants.MEDIUM_POSITION + 3000);
        } else {
            arm.raise(Constants.TOP_POSITION + 1000);
        }
        arm.score();
        Utils.sleep(1000);
        retract = true;
        drive.moveToPosition(new Vector2D(30,-17), VELOCITY);

        while(opModeIsActive());

    }


    private enum currentCycleState{
        PRELOAD, STRAFING_IN, INTAKING, LEAVING, SCORING
    }
    private void beginTiming() {
        initialTime = time;
        while(opModeIsActive()){
            if(time - initialTime > 25){
                executeArmCommands = false;
                drive.seteStop(true);
            }
            if(time - initialTime > 10 && state == currentCycleState.PRELOAD){
                drive.seteStop(true);            //we prob want the robot to stop on the preload case too because that means something catastrphic happened
            }
        }
    }

    @Override
    protected void onStop() {
        executeArmCommands = false;
        localizer.stopThread();

    }
}
