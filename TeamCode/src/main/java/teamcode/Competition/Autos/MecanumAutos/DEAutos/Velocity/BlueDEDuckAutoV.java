package teamcode.Competition.Autos.MecanumAutos.DEAutos.Velocity;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

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
import teamcode.common.Utils;

@Autonomous(name = "Blue DE Duck")
public class BlueDEDuckAutoV extends AbstractOpMode {


    /*TODO much like alot of the code in this codebase this was written at 2:30 am and after a lot of caffiene,
        TODO has not been field tested as of Friday morning

         */
    private OpenCvWebcam webcam;

    private EndgameSystems system;
    private MecanumDriveTrain drive;
    private ArmSystem arm;

    private volatile boolean[] flags;

    private Thread armCommands;

    private MecanumBarcodePipeline.BarcodePosition position;

    private PIDFCoefficients coefficients = new PIDFCoefficients(2,0.5,1.0,0);

    @Override
    protected void onInitialize() {
        system = new EndgameSystems(hardwareMap, true);
        arm = new ArmSystem(hardwareMap, false);
        drive = new MecanumDriveTrain(hardwareMap, false, system, arm, coefficients);
        flags = new boolean[]{false, false, false, false, false};
        armCommands = new Thread(){
            public void run(){
                while(!flags[3]);
                if(position == MecanumBarcodePipeline.BarcodePosition.LEFT){
                    arm.raise(Constants.BOTTOM_POSITION);
                }else if(position == MecanumBarcodePipeline.BarcodePosition.CENTER){
                    arm.raise(Constants.MEDIUM_POSITION + 1000);
                }else{
                    arm.raise(Constants.TOP_POSITION );
                }
                while (!flags[0]);

                arm.retract();

                while(!flags[1]);
                arm.raise(Constants.TOP_POSITION);
                while(!flags[2]);
                arm.score();
                Utils.sleep(250);
                arm.retract();
                while(opModeIsActive());
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

    private final double VELOCITY = 10;
    @Override
    protected void onStart() {
        webcam.stopStreaming();
        armCommands.start();


        //score the preload
        drive.moveDistanceDEVelocity(400, 0, VELOCITY);
        Utils.sleep(100);
        drive.rotateDistanceDE(-75, 4);
        Utils.sleep(100);
        drive.moveDistanceDEVelocity(1500, -90, VELOCITY / 2.0);
        Utils.sleep(100);
        drive.moveDistanceDEVelocity(400, 90, VELOCITY /2.0);

        flags[3] = true;
        Utils.sleep(100);
        drive.moveDistanceDEVelocity(300, 180, VELOCITY);
        if(position == MecanumBarcodePipeline.BarcodePosition.LEFT){
            arm.runConveyorPos(0.8, 2000);
        }else{
            Debug.log("her");
            arm.score();
        }
        Utils.sleep(200);

        //move away from hub and to carousel
        drive.moveDistanceDEVelocity(880, 0, VELOCITY);
        flags[0] = true;
        Utils.sleep(100);
        drive.rotateDistanceDE(160, -4);
        Utils.sleep(100);
        drive.strafeDistanceSensor(VELOCITY, 0);
        Utils.sleep(100);
        drive.moveDistanceDEVelocity(400,  90, VELOCITY / 2.0);
        Utils.sleep(100);
        //drive.driveColorSensorBlue(2);
        drive.moveDistanceDEVelocity(400, 0, 4.0); //calculated angle is 30
        drive.driveColorSensorBlue(2);
        Debug.log("second");
        Utils.sleep(100);
        drive.moveDistanceDEVelocity(350, 0, 2.0); //calculated angle is 30
        Debug.log("move");
        //drive.moveDistanceDEVelocity(100, 90, 5.0);
        //spin duck and run intake
        arm.lowerLinkage();
        arm.intakeDumb(1.0);
        system.scoreDuckAuto();
        Utils.sleep(500);
        arm.preScoreDuck();

        //score the duck
        drive.rotateDistanceDE(-105, 4);
        arm.intakeDumb(0);
        Utils.sleep(100);

        drive.moveDistanceDEVelocity(1600, -90, VELOCITY);

        flags[1] = true;
        drive.moveDistanceDEVelocity(700, 180, VELOCITY / 2.0);
        Utils.sleep(100);
        flags[2] = true;

        //park, go for partial because full is near impossible lmao,
        // could add a strafe and attempt it if extra time?
        drive.moveDistanceDEVelocity(900, 0, VELOCITY);

        while(opModeIsActive());
    }

    @Override
    protected void onStop() {
        drive.brake();
        drive.cleanup();
    }
}
