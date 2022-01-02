package teamcode.Competition.Autos.MecanumAutos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import teamcode.Competition.Pipeline.BarcodePipeline3;
import teamcode.Competition.Subsystems.ArmSystem;
import teamcode.Competition.Subsystems.EndgameSystems;
import teamcode.common.AbstractOpMode;
import teamcode.common.Constants;
import teamcode.common.Localizer;
import teamcode.common.MecanumDriveTrain;
import teamcode.common.PositionStuff.Pose;
import teamcode.common.Utils;
import teamcode.common.Vector2D;
import teamcode.common.WestCoastDriveTrain;

@Autonomous(name="RedAutoFreight")
public class MecanumFreightAuto extends AbstractOpMode {

    MecanumDriveTrain drive;
    ArmSystem arm;
    Localizer localizer;

    OpenCvWebcam webcam;
    BarcodePipeline3.BarcodePosition position;

    Thread secondaryFunctionsThread;

    private final int FREIGHT = 1;
    private final double VELOCITY = 15;
    private final double OMEGA = 0.4;


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

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        WebcamName wc = hardwareMap.get(WebcamName.class, "Webcam");

        // W/ or W/ out live preview
        webcam = OpenCvCameraFactory.getInstance().createWebcam(wc, cameraMonitorViewId);
        BarcodePipeline3 pipeline = new BarcodePipeline3();
        pipeline.setSide(BarcodePipeline3.Side.RED);
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
            Thread.currentThread().sleep(1000);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }

        arm.raise(Constants.TOP_POSITION);
        while(!retract && opModeIsActive());
        Utils.sleep(250);
        arm.retract();
        retract = false;
        for(int i = 0; i < FREIGHT; i++){
            while(!resetSensors && opModeIsActive()){}
            localizer.manualZero(false);
            resetSensors = false;

            while(!intake && opModeIsActive());
            boolean isStop = arm.intakeAuto(1);
            drive.setEnvironmentalTerminate(true);
            if(isStop){
                drive.seteStop(true);
            }
            intake = false;

            while(!extend && opModeIsActive());
            arm.raise(Constants.TOP_POSITION);
            extend = false;

            while(!retract && opModeIsActive());
            Utils.sleep(250);
            arm.retract();
            retract = false;

        }
    }

    @Override
    protected void onStart() {
        localizer.start();
        secondaryFunctionsThread.start();
        drive.moveToPosition(new Vector2D(24,24), VELOCITY);
        drive.rotateDistance(Math.toRadians(180), OMEGA);
        arm.score();
        retract = true;
        drive.rotateDistance(Math.toRadians(90), -OMEGA);
        //starting path
        for(int i = 0; i < FREIGHT; i++) {
            resetSensors = true;
            drive.strafeDistanceSensor(VELOCITY);
            localizer.resumeUpdateCycles();
            intake = true;
            drive.moveToPosition(new Vector2D(24,0), VELOCITY); //replace this with seekCubes() if it works
            drive.moveToPosition(new Vector2D(0, 0), -VELOCITY);
            extend = true;
            drive.moveToPosition(new Vector2D( 0, 24.5), VELOCITY); //replace this with a distance sensor command?
            drive.moveToRotation(Math.toRadians(45), OMEGA);
            arm.score();
            drive.moveToRotation(Math.toRadians(0), -OMEGA);
            retract = true;
        }

    }

    @Override
    protected void onStop() {

    }
}
