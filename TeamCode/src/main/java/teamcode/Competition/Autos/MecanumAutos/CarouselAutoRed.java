package teamcode.Competition.Autos.MecanumAutos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.openftc.easyopencv.OpenCvWebcam;

import teamcode.Competition.Pipeline.BarcodePipeline3;
import teamcode.Competition.Subsystems.ArmSystem;
import teamcode.common.AbstractOpMode;
import teamcode.common.Constants;
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
        while(!opModeIsActive()){
            position = BarcodePipeline3.BarcodePosition.LEFT;
            //telemetry.addData("", position);
            //telemetry.update();
        }
    }



    volatile boolean resetSensors, intake, extend, retract;
    //contingency plan for 0 freight, we set this to fencepost or comment out the intake section of it
    private void secondaryFunctionsSequence(){
        try {
            Thread.currentThread().sleep(1500);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }

        if(position == BarcodePipeline3.BarcodePosition.RIGHT) {
            arm.raise(Constants.BOTTOM_POSITION - 1500);
        }else if(position == BarcodePipeline3.BarcodePosition.CENTER){
            arm.raise(Constants.MEDIUM_POSITION + 3000);
        }else{
            arm.raise(Constants.TOP_POSITION + 3000);
        }
        while(!retract && opModeIsActive());
        Utils.sleep(250);
        arm.retract();
        retract = false;

    }

    @Override
    protected void onStart() {
        localizer.start();
        secondaryFunctionsThread.start();
        telemetry.clear();
        drive.moveToPosition(new Vector2D(30,5),  VELOCITY);

        //drive.moveToPosition(new Vector2D(20, 16), VELOCITY);
        drive.rotateDistance(Math.toRadians(90), OMEGA);
        //drive.moveToPosition(new Vector2D(18.5,15), -VELOCITY);
        arm.score();
        Utils.sleep(1000);
        retract = true;
        drive.rotateDistance(Math.toRadians(90), -OMEGA);
//        //starting path


//        if(state != currentCycleState.INTAKING && state != currentCycleState.LEAVING){
//
//            drive.seteStop(false);
//        }
//        if(state == currentCycleState.STRAFING_IN || state == currentCycleState.SCORING){
//            drive.rotateDistance(Math.toRadians(0), OMEGA);
//            localizer.manualZero(false);
//            drive.strafeDistanceSensor(VELOCITY);
//            localizer.resumeUpdateCycles();
//            drive.moveToPosition(new Vector2D(24,0), VELOCITY);
//        }

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
        localizer.stopThread();

    }
}
