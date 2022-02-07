package teamcode.Competition.Autos.MecanumAutos.DEAutos;

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
import teamcode.common.MecanumDriveTrain;


@Autonomous(name="Blue DE Warehouse")
public class BlueDEWarehouseAuto extends AbstractOpMode {

    MecanumDriveTrain drive;
    ArmSystem arm;
    EndgameSystems system;
    volatile boolean[] flags;
    Thread armCommands;
    OpenCvWebcam webcam;
    MecanumBarcodePipeline.BarcodePosition position;


    @Override
    protected void onInitialize() {
        system = new EndgameSystems(hardwareMap, false);
        drive = new MecanumDriveTrain(hardwareMap, true, system);
        arm = new ArmSystem(hardwareMap, false);
        Debug.log("here");
        flags = new boolean[]{false, false, false, false, false};
        armCommands = new Thread(){
            public void run(){

                    while(!flags[0]);
                    if(position == MecanumBarcodePipeline.BarcodePosition.LEFT) {
                        arm.runConveyorPos(1.0, 2000);
                    }else if(position == MecanumBarcodePipeline.BarcodePosition.CENTER){
                        arm.moveSlide( 1.0, Constants.MEDIUM_POSITION + 1000);
                    }else{
                        arm.moveSlide(1.0, Constants.TOP_POSITION + 1000);
                    }
                    flags[0] = false;
                    while(!flags[1]);
                    arm.score();
                    try {
                        Thread.currentThread().sleep(500);
                    } catch (InterruptedException e) {
                        e.printStackTrace();
                    }
                    arm.retract();
                    flags[1] = false;
                while(opModeIsActive()); //this was a mistake I made in my inital one too,
                // you want this here as a buffer so that the thread doesnt prematurely end vs what
                //I had which looped the threads actions and because the flags were all true there was no buffer time
            }
        };

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
        while(!opModeIsActive()){
            position = pipeline.getPos();
            telemetry.addData("", position);
            telemetry.update();
        }
    }

    @Override
    protected void onStart() {
        armCommands.start();
        arm.idleServos();
        /*drive.moveDistanceDE(400, 0, 0.3, 0);
        drive.rotateDistanceDE(-90, 0.3);
        drive.moveDistanceDE(1400, -90, 0.3, 0.2);
        drive.moveDistanceDE(200, -90, 0.3, 0.2);
        flags[0] = true;
        drive.moveDistanceDE(600, -180, 0.3, 0);
        flags[1] =true;
        drive.moveDistanceDE(600, 0, 0.3, 0);


        drive.goToAllianceHub;
        drive.score;
        drive.park;

         */

        drive.moveDistanceDE(500, 0, 0.3, 0);
        flags[0] = true;
        drive.rotateDistanceDE(120, 0.3);
        sleep(1000);
        drive.moveDistanceDE(440, -180, 0.3, 0);
        flags[1] = true;
        sleep(1000);
        drive.rotateDistanceDE(90, 0.3);
        sleep(1000);
<<<<<<< Updated upstream
        drive.moveDistanceDE(1700, -90, 0.6, 0);
=======
        drive.moveDistanceDE(1700, -90, 0.3, 0);
>>>>>>> Stashed changes
        sleep(1000);
        drive.moveDistanceDE(1200, 0, 0.3, 0);


        while(opModeIsActive());

    }

    @Override
    protected void onStop() {

    }
}
