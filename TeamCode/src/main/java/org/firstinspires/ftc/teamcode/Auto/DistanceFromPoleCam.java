package org.firstinspires.ftc.teamcode.Auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Common.Constants;
import org.firstinspires.ftc.teamcode.Common.MecanumDrive;
import org.firstinspires.ftc.teamcode.Common.ScoringSystem;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

@Autonomous @Config
public class DistanceFromPoleCam extends LinearOpMode {
    MecanumDrive drive;
    NormalizationTesting detect;
    ScoringSystem score;
    Constants constants;
    OpenCvWebcam webcam;

    private double properCX = 67;
    public static int positive_negative = 1;
    public static int turnDenom = 4;

    @Override
    public void runOpMode() throws InterruptedException {
        detect = new NormalizationTesting(telemetry);
        drive = new MecanumDrive(hardwareMap, telemetry);
        score = new ScoringSystem(hardwareMap, telemetry);
        constants = new Constants();

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        webcam.setPipeline(detect);

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


        // ftc dashboard
        FtcDashboard.getInstance().startCameraStream(webcam, 0);
        telemetry.addData("Status", "Initialized");

        telemetry.update();

        // code to turn servo of cam
        score.setCamPosition(constants.getStrafeCamPos());
//        while (opModeInInit()) {
//            score.setCamPosition(position);
//        }
        waitForStart();

        // go forward next to pole
        drive.goToPositionPID(1100, "go forward next to pole");
        // turn to left 45 degrees to medium pole
        drive.turn(-Math.PI / 4, 0);
        // go to pole a bit
        drive.goToPosition(0.3, 0.3, 0.3, 0.3, 100, "go forward some to pole");
        // camera position correction
        if (detect.getcX() < properCX - 5 || detect.getcX() > properCX + 5) {
            while (detect.getcX() < properCX - 5) {
                // strafe to the right
//                drive.goToPosition(0.2, -0.2, -0.2, 0.2);
                drive.goToPosition(-0.2, 0.2, 0.2, -0.2);

            }
            while (detect.getcX() > properCX + 5) {
                // strafe to the left (change fr and bl)
                drive.goToPosition(0.2, -0.2, -0.2, 0.2);

//                drive.goToPosition(-0.2, 0.2, 0.2, -0.2);

            }
            drive.goToPosition(0,0,0,0);
        }

//        3700 - 3800
//        while (detect.getBoundArea() >= 3850.0 || detect.getcX() >= 18) {
//            drive.goToPosition(0.1, 0.1, 0.1, 0.1);

//        }
//        drive.goToPosition(0, 0, 0, 0);


    }


}
