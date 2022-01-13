package teamcode.Competition.Autos.MecanumAutos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import teamcode.Competition.Pipeline.Calibrators.MecanumAutoPosition;
import teamcode.Competition.Pipeline.MecanumPipeline.MecanumBarcodePipeline;
import teamcode.Competition.Pipeline.TankPipeline.BarcodePipeline3;
import teamcode.Competition.Subsystems.ArmSystem;
import teamcode.common.AbstractOpMode;
import teamcode.common.Constants;
import teamcode.common.Localizer;
import teamcode.common.MecanumDriveTrain;
import teamcode.common.PositionStuff.Pose;
import teamcode.common.Utils;
import teamcode.common.Vector2D;

@Autonomous(name="BlueAutoFreight")
public class MecanumFreightAutoBlue extends AbstractOpMode {

    MecanumDriveTrain drive;
    ArmSystem arm;
    Localizer localizer;
    MecanumAutoPosition autoPos;

    OpenCvWebcam webcam;
    //BarcodePipeline3.BarcodePosition position;
    MecanumBarcodePipeline.BarcodePosition position;

    Thread secondaryFunctionsThread, timerThread;

    private final int FREIGHT = 0;
    private final double VELOCITY = 15;
    private final double OMEGA = 0.4;

    double deltaTime, initialTime;
    volatile currentCycleState state;
    volatile boolean executeArmCommands;


    @Override
    protected void onInitialize() {
        localizer = new Localizer(new Pose(0,0,0), hardwareMap);
        drive = new MecanumDriveTrain(hardwareMap, localizer, false);
        arm = new ArmSystem(hardwareMap, false);
        //autoPos = new MecanumAutoPosition(MecanumBarcodePipeline.Side.BLUE);

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

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        WebcamName wc = hardwareMap.get(WebcamName.class, "Webcam");

        // W/ or W/ out live preview
        webcam = OpenCvCameraFactory.getInstance().createWebcam(wc, cameraMonitorViewId);
        MecanumBarcodePipeline pipeline = new MecanumBarcodePipeline();
        pipeline.setSide(MecanumBarcodePipeline.Side.BLUE);
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
        localizer.lowerOdo();
        executeArmCommands = true;
        while(!opModeIsActive()){
            position = pipeline.getPos();
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

        if(position == MecanumBarcodePipeline.BarcodePosition.LEFT) {
            arm.raise(Constants.BOTTOM_POSITION - 2500);
        }else if(position == MecanumBarcodePipeline.BarcodePosition.CENTER){
            arm.raise(Constants.MEDIUM_POSITION + 3000);
        }else{
            arm.raise(Constants.TOP_POSITION + 3000);
        }
        while(!retract && opModeIsActive());
        Utils.sleep(250);
        arm.moveSlide(-1,0);
        retract = false;
        for(int i = 0; i < FREIGHT; i++){


            while(!intake && opModeIsActive());
            if(executeArmCommands) {
                //boolean isStop = arm.intakeAuto(1);
                drive.setEnvironmentalTerminate(true);
//                if (isStop) {
//                    drive.seteStop(true);
//                }
//                intake = false;
            }

            while(!extend && opModeIsActive());
            if(executeArmCommands) {
                arm.raise(Constants.TOP_POSITION + 3000);
                extend = false;
            }

            while(!retract && opModeIsActive());
            if(executeArmCommands) {
                Utils.sleep(250);
                arm.moveSlide(-1,0);
                retract = false;
            }
            if(!executeArmCommands){
                break;
            }
        }
    }

    @Override
    protected void onStart() {
        localizer.start();
        secondaryFunctionsThread.start();
        telemetry.clear();
        drive.moveToPosition(new Vector2D(15,15),  VELOCITY);
        drive.rotateDistance(Math.toRadians(180), OMEGA);
        drive.moveToPosition(new Vector2D(19,12), VELOCITY);

        //drive.moveToPosition(new Vector2D(20, 16), VELOCITY);

        //drive.moveToPosition(new Vector2D(18.5,15), -VELOCITY);
        arm.score();
        Utils.sleep(1000);
        retract = true;
        drive.rotateDistance(Math.toRadians(-105), OMEGA);
//        //starting path
//        for(int i = 0; i < FREIGHT; i++) {
//            state = currentCycleState.STRAFING_IN;
            drive.strafeDistanceSensor(-0.7, Math.toRadians(0));
            localizer.manualZero(true);

            //Utils.sleep(1000);
            intake = true;
            state = currentCycleState.INTAKING;
            drive.moveToPosition(new Vector2D(48,0), 2* VELOCITY); //replace this with seekCubes() if it works
//            state = currentCycleState.LEAVING;
            //drive.strafeDistanceSensor(-0.7, 0);
//            drive.moveToPosition(new Vector2D(-6, 0), VELOCITY);
//            extend = true;
//            state = currentCycleState.SCORING;
//            drive.moveToPosition(new Vector2D( -15, 15), VELOCITY); //replace this with a distance sensor command?
//            drive.moveToPosition(new Vector2D( 15, 18), -VELOCITY); //replace this with a distance sensor command?
//
//            drive.rotateDistance(Math.toRadians(-90), -OMEGA);
//            arm.score();
//            drive.rotateDistance(Math.toRadians(0), OMEGA);
//            retract = true;
        //}

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
