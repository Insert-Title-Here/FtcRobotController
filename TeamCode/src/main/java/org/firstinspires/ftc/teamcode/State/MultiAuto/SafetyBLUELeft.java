package org.firstinspires.ftc.teamcode.State.MultiAuto;


import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.State.Auto.ContourMultiScoreLeft2;
import org.firstinspires.ftc.teamcode.State.Common.Constants;
import org.firstinspires.ftc.teamcode.State.Common.MecanumDrive;
import org.firstinspires.ftc.teamcode.State.Common.ScoringSystem;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.concurrent.atomic.AtomicBoolean;


@Autonomous
public class SafetyBLUELeft extends LinearOpMode {
    // instantiating
    Thread liftThread;
    MecanumDrive drive;
    ContourMultiScoreLeft2 detect1;
    ScoringSystem score;
    Constants constants;
    OpenCvWebcam webcam;
    AtomicBoolean liftTurn, liftCone, safePark;
    volatile boolean lifting = false;
    long time;
    long onTimeout;

    // ftc dashboard values + properCX
    private double properCX = 165; //67  170
    private double properCXHigh = 160   ; //67

    @Override
    public void runOpMode() throws InterruptedException {
        // value initializing
        detect1 = new ContourMultiScoreLeft2(telemetry);
        drive = new MecanumDrive(hardwareMap, telemetry);
        //sets Auto driving to break instead of float
        
        score = new ScoringSystem(hardwareMap, telemetry);
        constants = new Constants();
        liftTurn = new AtomicBoolean();
        liftCone = new AtomicBoolean();
        safePark = new AtomicBoolean();
        liftCone.set(false);
        liftTurn.set(false);
        safePark.set(false);


        // camera setup
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        webcam.setPipeline(detect1);

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {

            @Override
            public void onOpened() {
                webcam.startStreaming(320, 176, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addData("Camera Init Error", errorCode);
                telemetry.update();
            }
        });

        // thread for keeping lift up
        liftThread = new Thread() {
            @Override
            public void run() {
                while (opModeIsActive()) {
                    if ((score.getEncoderPosition() > 30) && !lifting) {
                        score.setPower(constants.getSteadyPow());
                    }
                    if (liftTurn.get()) {
                        //score.stackDown();
                        //score.goToPosition(constants.getStackHeight() - sum - 15, 0.6);
                        liftTurn.set(false);
                    }
                    if (liftCone.get()) {
                        drive.goToPosition(0.4, 0.4, 0.4, 0.4, 15, "back a bit");
                    }


//
                    telemetry.addData("imu", drive.currentAngle());
                    telemetry.update();
                }

            }
        };

        //properCX = detect1.getBoundWidth();
        // code to turn servo of cam
        //score.setCamPosition(constants.getSleeveCamPos());
        //detect1.park = true;

        // ftc dashboard setup
        FtcDashboard.getInstance().startCameraStream(webcam, 0);
        telemetry.addData("Status", "Initialized");

        telemetry.update();

        //close claw
        score.setClawPosition(constants.getClawClosePos()); //TODO

        // turn servo of cam forward for sleeves
        score.setCamPosition(constants.getSleeveCamPos());


        waitForStart();
        detect1.park = false;
        // turn servo of cam forward for poles
        //score.setCamPosition(constants.getStrafeLowCamPos());

        liftThread.start();

        //lift claw a little bit
        score.goToPosition(50, 0.8);
        sleep(100);
        // go forward next to pole
        drive.goToPosition(0.74, 0.7, 0.74, 0.7, 1280, "go forward next to pole");
        sleep(500);
        // turn to left 45 degrees to medium pole
        drive.absTurnPID(Math.PI / 2);

        score.setClawPosition(constants.getClawOpenPos());

        // PARK ------------------------------------------------------>

        //moves robot to correct parking position
        if (detect1.getParkPosition() == ContourMultiScoreLeft2.ParkingPosition.LEFT) {
            // go forward
            drive.goToPosition(0.74, 0.7, 0.74, 0.7, 650, "go forwards");
            sleep(100);
            // strafe left
            drive.goToPosition(-0.7, 0.7, 0.7, -0.7, 100, "strafe left");

        } else if (detect1.getParkPosition() == ContourMultiScoreLeft2.ParkingPosition.CENTER) {
            // go forward
            drive.goToPosition(0.74, 0.7, 0.74, 0.7, 80, "go forwards");
            sleep(100);

            // strafe left
            drive.goToPosition(-0.7, 0.7, 0.7, -0.7, 200, "strafe left");
        } else {
            // go backwards
            drive.goToPosition(-0.7, -0.7, -0.7, -0.7, 390, "go backwards");
            sleep(100);

            // strafe left
            drive.goToPosition(-0.7, 0.7, 0.7, -0.7, 200, "strafe left");


        }
        score.goToPosition(0, 0.85);

            liftThread.stop();
    }
    public void scoreConePreload(int fl, int fr, int bl, int br) {
        /*
        // move arm medium
        lifting = true;
        score.goToPositionPID(constants.getHeightMed()+30)+10, 0.85);
        lifting = false;
        */

        //begin thread for maintaining height of slides

        onTimeout = System.currentTimeMillis();
        //3700 - 3800
        drive.goToPosition(0.12, 0.12, 0.12, 0.12);
        sleep(100);
        // old: 7900     (9100 new)  (6800 new)  7400

        while (detect1.getBoundArea() <= 5700.0 || detect1.getBoundArea() >= 8400) { //7200
            if (detect1.getBoundArea() >= 5700.0 && detect1.getBoundArea() <= 8400 && detect1.getDistance() <= 5.5/*|| detect1.getcX() <= 18*/) {
                drive.goToPosition(0, 0, 0, 0);
                break;
            } else if ((System.currentTimeMillis() - onTimeout) / 1000 > 3) {
                break;
            }

        }




        sleep(200);
        score.setClawPosition(constants.getClawOpenPos());

        //move back from pole
        drive.goToPosition(-0.5, -0.5, -0.5, -0.5, fl, "move back from pole");

        //lower cone ontto pole
        score.goToPosition(0, 0.6);




    }
}
