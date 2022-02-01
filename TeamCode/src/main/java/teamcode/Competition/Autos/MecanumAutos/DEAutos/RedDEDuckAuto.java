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
import teamcode.common.Utils;

@Autonomous(name="Red DE Duck")
public class RedDEDuckAuto extends AbstractOpMode {

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
            public void run() {

                while (!flags[0]) ;
                if (position == MecanumBarcodePipeline.BarcodePosition.LEFT) {
                    arm.moveSlideAuto(1.0, Constants.BOTTOM_POSITION);
                    Utils.sleep(250);
                    arm.runConveyorPos(0.8, 2000);
                } else if (position == MecanumBarcodePipeline.BarcodePosition.CENTER) {
                    arm.moveSlideAuto(1.0, Constants.MEDIUM_POSITION + 1500);
                } else {
                    arm.moveSlideAuto(1.0, Constants.TOP_POSITION);
                }
                while (!flags[1]) ;
                arm.score();
                try {
                    Thread.currentThread().sleep(500);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
                arm.retract();
                while (!flags[2]) ;

                Utils.sleep(500);
                arm.moveSlideAuto(1.0, Constants.TOP_POSITION);
                while (!flags[3]) ;
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
        webcam.stopStreaming();
        armCommands.start();
        //arm.idleServos();
        drive.moveDistanceDE(400, 0, 0.3, 0);
        drive.rotateDistanceDE(75, 0.3);
        Utils.sleep(500);
        drive.moveDistanceDE(1400, 90, 0.3, 0.2);
        //drive.moveDistanceDE(200, -90, 0.3, 0.2);
        flags[0] = true;
        drive.moveDistanceDE(400, -180, 0.3, 0.2);
        flags[1] =true;
        drive.moveDistanceDE(300, 0, 0.3, 0.2);
        drive.moveDistanceDE(2200, -72.69, 0.3, 0.2);
        drive.rotateDistanceDE(75, 0.2);
        drive.moveDistanceDE(100, -90, 0.2, 0.2);

        drive.setStrafe(-0.01);
        system.scoreDuckAuto();
        drive.moveDistanceDE(200, 90, 0.3, 0.2);
        drive.rotateDistanceDE(165, 0.3);
        arm.lowerLinkage();
        Utils.sleep(500);
        drive.moveDistanceDENoErrorCorrection(150, 0, 0.2);
        //drive.setPower(0.2,0.2,0.2,0.2);
        arm.intakeDumb(1.0);
        drive.moveDistanceDENoErrorCorrection(1000, -90, 0.2);
        //drive.moveDistanceDENoErrorCorrection(200, 0, 0.3);
        drive.moveDistanceDENoErrorCorrection(800, 90, 0.2);
        drive.brake();
        arm.preScoreDuck();
        arm.intakeDumb(0);
        drive.rotateDistanceDE(75, 0.3);
        Utils.sleep(200);

        drive.moveDistanceDE(1800, 90, 0.3,0.2);
        flags[2] = true;
        drive.moveDistanceDENoErrorCorrection(800, 180, 0.3);
        //drive.rotateDistanceDE(75, 0.3);
        flags[3] = true;
        Utils.sleep(250);
        //drive.rotateDistanceDE(105, 0.3);
        drive.moveDistanceDE(600, 0, 0.3,0.2);



        //drive.smartDuck(true);
        while(opModeIsActive());

    }

    @Override
    protected void onStop() {
        drive.cleanup();

    }
}
