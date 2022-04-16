package teamcode.Competition.Autos.MecanumAutos.DEAutos.Velocity.WorldsAuto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.checkerframework.checker.units.qual.A;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import teamcode.Competition.Pipeline.MecanumPipeline.MecanumBarcodePipeline;
import teamcode.Competition.Pipeline.MecanumPipeline.TapePipeline;
import teamcode.Competition.Subsystems.ArmSystem;
import teamcode.Competition.Subsystems.EndgameSystems;
import teamcode.common.AbstractOpMode;
import teamcode.common.Constants;
import teamcode.common.Debug;
import teamcode.common.MecanumDriveTrain;
import teamcode.common.Utils;
import teamcode.common.Vector2D;
import teamcode.test.MasonTesting.DuckPipeline;

@Autonomous(name="dook red")
public class IntakeDuckNew extends AbstractOpMode {
    MecanumDriveTrain drive;
    ArmSystem system;
    EndgameSystems systems;
    DuckPipeline duck;
    private OpenCvWebcam webcam;
    private TapePipeline.BarcodePosition position;
    Thread armThread;
    volatile boolean[] flags = new boolean[]{false, false, false, false};
    private PIDFCoefficients coefficients = new PIDFCoefficients(5,0.5,1.0,0);


    @Override
    protected void onInitialize() {
        duck = new DuckPipeline();
        system = new ArmSystem(hardwareMap, false);
        systems = new EndgameSystems(hardwareMap, false);
        drive = new MecanumDriveTrain(hardwareMap, true, systems, system, coefficients);
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        WebcamName wc = hardwareMap.get(WebcamName.class, "Webcam");

        // W/ or W/ out live preview
        webcam = OpenCvCameraFactory.getInstance().createWebcam(wc, cameraMonitorViewId);
        TapePipeline pipeline = new TapePipeline();
        pipeline.setSide(TapePipeline.Side.RED);
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
        armThread = new Thread(){
            @Override
            public void run(){
                while(!flags[0]);
                if(position == TapePipeline.BarcodePosition.LEFT) {
                    system.raise(Constants.BOTTOM_POSITION);
                }else if(position == TapePipeline.BarcodePosition.CENTER){
                    system.raise(Constants.MEDIUM_POSITION + 3000);
                }else{
                    system.raise(Constants.TOP_POSITION);
                }
                while(!flags[1]);


                system.retract();
                while(!flags[2]);
                system.raise(Constants.TOP_POSITION + 3000);
                Utils.sleep(400);
                system.retract();
                while(!flags[3]);



            }
        };
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
        armThread.start();
        system.actuateWinchStop(1.0);
        drive.moveDistanceDEVelocity(1000, -60, 2 * VELOCITY);
        //Utils.sleep(200);

        Utils.sleep(200);
        //drive.driveColorSensorWarehouse(6);
        drive.moveDistanceDEVelocity(750, 0, VELOCITY / 2.0);
        Utils.sleep(400);
        drive.rotateDistanceDE(90, 3);

        //extend
        flags[0] = true;
        Utils.sleep(200);
        drive.moveDistanceDEVelocity(800, 180, VELOCITY);
        //score
        if(position == TapePipeline.BarcodePosition.LEFT){
            system.runConveyorPos(0.5, 2000);
        }else {
            system.score();
        }
        Utils.sleep(200);
        drive.moveDistanceDEVelocity(200, 90, VELOCITY ); //400


        drive.rotateDistanceDE(150, 24);
        flags[1] = true;
        //Utils.sleep(200);
        drive.strafeDistanceSensor(30, -Math.PI / 6.0);
        system.setIsDuck(true);
        Utils.sleep(200);
        //drive.moveDistanceDEVelocity(300, 180, VELOCITY);

        drive.moveDistanceDEVelocity(300, -90, VELOCITY);
        Utils.sleep(200);
       // drive.rotateDistanceDE(150, -6);
        Utils.sleep(200);
        drive.moveDistanceDEVelocity(600, 0, VELOCITY);//
        //drive.driveColorSensorWarehouse(4);
        drive.moveDistanceDEVelocity(400, 0, VELOCITY / 2.0);
        Utils.sleep(200);

        drive.duck();
        system.lowerLinkage();
        system.intakeDumb(1.0);
        drive.setPower(0.05,0.05,0.05,0.05);

        systems.scoreDuckAuto();
        Utils.sleep(250);
        drive.moveDistanceDEVelocity(600, 180, VELOCITY);
        Utils.sleep(250);
        drive.rotateDistanceDE(90, 6);
        Utils.sleep(250);
        drive.moveDistanceDEVelocity(200, 0, VELOCITY);

    }

    private void scoreDuck(){
        drive.moveDistanceDEVelocity(300, -90, VELOCITY / 2.0);
        Utils.sleep(200);
        drive.moveDistanceDEVelocity(100, 0, VELOCITY / 2.0);
        Utils.sleep(200);
        drive.moveDistanceDEVelocity(800, -90, VELOCITY / 2.0); //400


        Utils.sleep(500);
        //drive.moveDistanceDEVelocity(100, -90, VELOCITY / 2.0);

        //drive.moveDistanceDEVelocity(420, 0, VELOCITY / 2.0);
        //Utils.sleep(500);
        system.preScore();

        drive.moveDistanceDEVelocity(100, 180, VELOCITY);
        Utils.sleep(200);
        system.intakeDumb(0);
        // drive.rotateDistanceDE(150, 24);
        //Utils.sleep(200);
        drive.moveDistanceDEVelocity(1400, 90, 2 * VELOCITY); //400
        Utils.sleep(200);
        drive.moveDistanceDEVelocity(700, 180,  VELOCITY * 2.0);
        Utils.sleep(200);
        drive.rotateDistanceDE(105, 12);
        Utils.sleep(200);

        //extend
        flags[2] = true;
        drive.moveDistanceDEVelocity(400, 180, 2 * VELOCITY);
        //score
        flags[3] = true;
        Utils.sleep(200);
        system.score();
        Utils.sleep(200);
        drive.moveDistanceDEVelocity(800, 0, 2 * VELOCITY);
        Utils.sleep(200);
        drive.moveDistanceDEVelocity(400, -90, 2 * VELOCITY ); //400


    }

        //drive.strafeDistanceSensor(6,0);




    @Override
    protected void onStop() {

    }
}
