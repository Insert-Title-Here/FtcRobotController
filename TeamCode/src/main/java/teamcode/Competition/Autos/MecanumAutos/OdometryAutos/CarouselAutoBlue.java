package teamcode.Competition.Autos.MecanumAutos.OdometryAutos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import teamcode.Competition.Pipeline.MecanumPipeline.MecanumBarcodePipeline;
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

@Autonomous(name="CarouselBlue")
public class CarouselAutoBlue extends AbstractOpMode {

    MecanumDriveTrain drive;
    ArmSystem arm;
    Localizer localizer;
    EndgameSystems system;

    OpenCvWebcam webcam;
    MecanumBarcodePipeline.BarcodePosition position;

    Thread secondaryFunctionsThread, timerThread;

    private final double VELOCITY = 15;
    private final double OMEGA = 0.4;

    double deltaTime, initialTime;
    volatile currentCycleState state;
    volatile boolean executeArmCommands;


    @Override
    protected void onInitialize() {
        system = new EndgameSystems(hardwareMap, true);
        localizer = new Localizer(new Pose(0,0,0), hardwareMap);
        drive = new MecanumDriveTrain(hardwareMap, localizer, false, system);
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
        extend = false;
        retract = false;
        spin = false;
        duckExit = false;
        while(!opModeIsActive()){
            position = pipeline.getPos();
            telemetry.addData("", position);
            telemetry.update();
        }
    }



    volatile boolean duckExit, spin, extend, retract;
    //contingency plan for 0 freight, we set this to fencepost or comment out the intake section of it
    private void secondaryFunctionsSequence(){



    }

    @Override
    protected void onStart() {
        webcam.stopStreaming();
        localizer.start();
        telemetry.clear();
        drive.moveToPosition(new Vector2D(9,0),  VELOCITY);

        //drive.moveToPosition(new Vector2D(20, 16), VELOCITY);
        drive.rotateDistance(Math.toRadians(105), OMEGA);
        drive.moveToPosition(new Vector2D(14,17), VELOCITY);
        drive.moveToPosition(new Vector2D(4.5,17), VELOCITY);
        drive.smartDuck(true);
        //drive.setStrafe(-0.1);
        drive.moveToPosition(new Vector2D(34,15), VELOCITY);
        double rotation = localizer.getCurrentState().getRotation();
        double delta = Math.toRadians(-100) - rotation;
        telemetry.clear();
        Debug.log(delta);
        drive.rotateDistance(Math.toRadians(90), -OMEGA);
        localizer.manualZero(true);
        drive.moveToPosition(new Vector2D(-27,0), VELOCITY);//15.25
        //drive.rotateDistance(Math.toRadians(-85), OMEGA);

        if (position == MecanumBarcodePipeline.BarcodePosition.LEFT) {
            arm.raise(Constants.BOTTOM_POSITION + 2500);
        } else if (position == MecanumBarcodePipeline.BarcodePosition.CENTER) {
            arm.raise(Constants.MEDIUM_POSITION + 2000);
        } else {
            arm.raise(Constants.TOP_POSITION);
        }
        arm.score();
        Utils.sleep(1000);

        drive.moveToPosition(new Vector2D(1,7), VELOCITY); //25
        arm.retract();
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
