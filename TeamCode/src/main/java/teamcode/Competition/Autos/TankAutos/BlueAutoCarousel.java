package teamcode.Competition.Autos.TankAutos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import teamcode.Competition.Pipeline.TankPipeline.CarouselPipeline;
import teamcode.Competition.Subsystems.ArmSystem;
import teamcode.Competition.Subsystems.EndgameSystems;
import teamcode.common.AbstractOpMode;
import teamcode.common.Constants;
import teamcode.common.Debug;
import teamcode.common.Localizer;
import teamcode.common.PositionStuff.Pose;
import teamcode.common.Utils;
import teamcode.common.Vector2D;
import teamcode.common.WestCoastDriveTrain;

@Disabled
@Autonomous(name="CarouselBlue TANK")
public class BlueAutoCarousel extends AbstractOpMode {
    WestCoastDriveTrain driveTrain;
    ArmSystem arm;
    EndgameSystems system; //carousel
    Localizer localizer;

    OpenCvWebcam webcam;
    CarouselPipeline.BarcodePosition position;
    private volatile boolean spin, endSpin;
    Thread carouselThread;

    @Override
    protected void onInitialize() {
        localizer = new Localizer(new Pose(0,0,0), hardwareMap);
        driveTrain = new WestCoastDriveTrain(hardwareMap, localizer);
        system = new EndgameSystems(hardwareMap, true);
        arm = new ArmSystem(hardwareMap, false);
        localizer.lowerOdo();

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        WebcamName wc = hardwareMap.get(WebcamName.class, "Webcam");

        // W/ or W/ out live preview
        webcam = OpenCvCameraFactory.getInstance().createWebcam(wc, cameraMonitorViewId);
        // camera = OpenCvCameraFactory.getInstance().createWebcam(wc);

        CarouselPipeline pipeline = new CarouselPipeline();
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
        spin = false;
        endSpin = false;
        carouselThread = new Thread(){
            @Override
            public void run(){
                for(int i = 0; i < 1; i++) {
                    while (!spin){
                    }
                    //system.runCarousel(-0.1);
                    system.scoreDuckAuto();
                    endSpin = true;
                    spin = false;
                }
            }
        };
        while(!opModeIsActive()){
            position = pipeline.getPos();
            //telemetry.addData("", position);
            //telemetry.update();
        }
    }

    @Override
    protected void onStart() {
        webcam.stopStreaming();
        telemetry.clear();
        localizer.start();
        carouselThread.start();

        driveTrain.moveToPosition(new Vector2D(-1, 6), 12, 0.5, false);
        driveTrain.rotateDistance(0.5, Math.toRadians(-120));

        if (position == CarouselPipeline.BarcodePosition.LEFT) {
            Debug.log("top");
            arm.raise(Constants.TOP_POSITION );
        } else if(position == CarouselPipeline.BarcodePosition.CENTER){
            Debug.log("mid");
            arm.raise(Constants.MEDIUM_POSITION);
        }
        driveTrain.moveToPosition(new Vector2D(-15.5, -22.5), -12, 0.5, false);
        driveTrain.rotateDistance(0.4, Math.toRadians(-155));
        if (position == CarouselPipeline.BarcodePosition.RIGHT) {
            Debug.log("low");
            arm.raise(Constants.BOTTOM_POSITION - 1000);
            arm.runConveyorPos(1, 2000);
        }
        arm.score();
        Utils.sleep(500) ;
        driveTrain.moveToPosition(new Vector2D(-15, -22), 12, 0.5, false);

        driveTrain.rotateDistance(-0.4, Math.toRadians(-105));
        driveTrain.moveToPosition(new Vector2D(-0, 10), 24, 0.5, false);
        arm.retract();
        driveTrain.rotateDistance(0.4, Math.toRadians(-120));
        driveTrain.moveToPosition(new Vector2D(-1, 17), 12, 0.5, false);
        driveTrain.rotateDistance(-0.4, Math.toRadians(-105));

        spin = true;
        driveTrain.setPower(-0.12,0);
        while(!endSpin){
        }
        driveTrain.rotateDistance(0.4, Math.toRadians(-120));



        // Utils.sleep(2000);



        while(opModeIsActive());

    }

    @Override
    protected void onStop() {
        localizer.stopThread();
        carouselThread.interrupt();
    }
}
