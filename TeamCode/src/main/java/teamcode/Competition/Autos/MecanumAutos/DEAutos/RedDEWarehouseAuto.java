package teamcode.Competition.Autos.MecanumAutos.DEAutos;

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
import teamcode.common.MecanumDriveTrain;

public class RedDEWarehouseAuto extends AbstractOpMode {
    private static final int FREIGHT = 0;
    private MecanumDriveTrain drive;
    private ArmSystem arm;
    private boolean[] flags;
    private Thread armCommands;
    private OpenCvWebcam webcam;
    private MecanumBarcodePipeline.BarcodePosition position;


    @Override
    protected void onInitialize() {
        drive = new MecanumDriveTrain(hardwareMap, true, null);
        arm = new ArmSystem(hardwareMap, false);
        Debug.log("here");
        flags = new boolean[]{false, false, false, false, false};
        armCommands = new Thread(){
            public void run(){

                while(!flags[0]);
                if(position == MecanumBarcodePipeline.BarcodePosition.LEFT){
                    arm.runConveyorPos(1,2000);
                }else if(position == MecanumBarcodePipeline.BarcodePosition.CENTER){
                    arm.moveSlide(1.0, Constants.MEDIUM_POSITION);
                }else {
                    arm.moveSlide(1.0, Constants.TOP_POSITION);
                }
                while(!flags[1]);
                arm.score();
                try {
                    Thread.currentThread().sleep(500);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
                arm.retract();
                while(!flags[2]);
                arm.moveSlide(1.0, Constants.TOP_POSITION);
                while(!flags[3]);
                arm.score();
                try {
                    Thread.currentThread().sleep(500);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
                arm.retract();
                while(opModeIsActive());
            }
        };

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        WebcamName wc = hardwareMap.get(WebcamName.class, "Webcam");

        // W/ or W/ out live preview
        webcam = OpenCvCameraFactory.getInstance().createWebcam(wc, cameraMonitorViewId);
        MecanumBarcodePipeline pipeline = new MecanumBarcodePipeline();
        pipeline.setSide(MecanumBarcodePipeline.Side.RED);
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
            telemetry.addData("", position);
            telemetry.update();
        }
    }

    @Override
    protected void onStart() {


    }

    @Override
    protected void onStop() {

    }
}
