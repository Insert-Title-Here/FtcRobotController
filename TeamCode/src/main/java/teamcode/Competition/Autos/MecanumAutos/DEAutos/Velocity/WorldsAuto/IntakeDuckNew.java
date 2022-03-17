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
import teamcode.Competition.Subsystems.ArmSystem;
import teamcode.Competition.Subsystems.EndgameSystems;
import teamcode.common.AbstractOpMode;
import teamcode.common.Constants;
import teamcode.common.MecanumDriveTrain;
import teamcode.common.Utils;
import teamcode.common.Vector2D;
import teamcode.test.MasonTesting.DuckPipeline;

@Autonomous(name="dook")
public class IntakeDuckNew extends AbstractOpMode {
    MecanumDriveTrain drive;
    ArmSystem system;
    EndgameSystems systems;
    DuckPipeline duck;
    private OpenCvWebcam webcam;
    private MecanumBarcodePipeline.BarcodePosition position;
    Thread armThread;


    @Override
    protected void onInitialize() {
        duck = new DuckPipeline();
        system = new ArmSystem(hardwareMap, false);
        systems = new EndgameSystems(hardwareMap, false);
        drive = new MecanumDriveTrain(hardwareMap, true, systems, system);
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
        armThread = new Thread(){
            public void run(){

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
        armThread.start();
        webcam.setPipeline(duck);
        drive.moveDistanceDEVelocity(1200, -45, VELOCITY);
        //Utils.sleep(200);

        Utils.sleep(200);
        //drive.driveColorSensorWarehouse(6);
        drive.moveDistanceDEVelocity(600, 0, VELOCITY / 2.0);
        Utils.sleep(200);
        drive.rotateDistanceDE(75, 4);

        //extend
        system.raise(Constants.TOP_POSITION);
        drive.moveDistanceDEVelocity(800, 180, VELOCITY);
        //score
        system.score();
        Utils.sleep(200);

        drive.rotateDistanceDE(160, 4);
        system.retract();

        //Utils.sleep(200);
        drive.strafeDistanceSensor(6,0);
        system.setIsDuck(true);
        //Utils.sleep(200);
        //drive.moveDistanceDEVelocity(300, 180, VELOCITY);

        drive.moveDistanceDEVelocity(250, -90, VELOCITY);
        drive.driveColorSensorWarehouse(4);
        drive.moveDistanceDEVelocity(300, 0, VELOCITY / 2.0);//
        drive.driveColorSensorWarehouse(4);
        drive.duck();
        system.lowerLinkage();
        system.intakeDumb(1.0);

        systems.scoreDuckAuto();
        Utils.sleep(1000);
        drive.moveDistanceDEVelocity(400, 180, VELOCITY / 2.0);
        NormalizedRGBA rgba = drive.getSensorRGBA();
        if(rgba.green < (0.9) && rgba.red < 0.9){
            double direction = duck.direction();
            while (!duck.isCentered()) {
                if(direction == 0){
                    break;
                }
                telemetry.addData("DIR: ", duck.direction());
                telemetry.addData("is", duck.isCentered());
                telemetry.update();
                drive.setStrafe(duck.direction() * 0.5);
            }
            drive.brake();
            //drive.moveDistanceDEVelocity(100, -90, VELOCITY / 2.0);
            if(direction != 0) {
                drive.driveColorSensorNoWarehouse(1);
            }
            drive.moveDistanceDEVelocity(800, 180, VELOCITY);
            drive.rotateDistanceDE(90, 4);

            //extend
            system.raise(Constants.TOP_POSITION);
            drive.moveDistanceDEVelocity(800, 180, VELOCITY);
            //score
            system.score();
            Utils.sleep(200);

            drive.rotateDistanceDE(160, 4);
            system.retract();


        }

        //drive.strafeDistanceSensor(6,0);


    }



    @Override
    protected void onStop() {

    }
}
