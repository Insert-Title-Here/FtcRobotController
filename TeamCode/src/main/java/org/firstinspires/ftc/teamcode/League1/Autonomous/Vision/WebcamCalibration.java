package org.firstinspires.ftc.teamcode.League1.Autonomous.Vision;

////import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.League1.Common.Constants;
import org.firstinspires.ftc.teamcode.League1.Subsystems.MecDrive;
import org.opencv.core.Scalar;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

@TeleOp
public class WebcamCalibration extends LinearOpMode {

    Servo cameraServo;
    OpenCvWebcam camera;
    CalibrationPipeline pipeline;
    MecDrive drive;
    boolean aFlag = true;
    boolean bFlag = true;
    int servoPos = 0;
    double recHUpper, recHLower, recSUpper, recSLower, recVUpper, recVLower, recYUpper, recYLower, recCrUpper, recCrLower, recCbUpper, recCbLower;
    Scalar colors;

    int yRange = 10;
    int crRange = 10;
    int cbRange = 10;
    int hRange = 10;
    int sRange = 10;
    int vRange = 3;

    @Override
    public void runOpMode() throws InterruptedException {
        cameraServo = hardwareMap.get(Servo.class, "camera");
        drive = new MecDrive(hardwareMap, false, telemetry);

        WebcamName webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);
        pipeline = new CalibrationPipeline(telemetry, drive, CalibrationPipeline.AutoSide.RED_RIGHT);

        camera.setPipeline(pipeline);

        cameraServo.setPosition(Constants.coneV2);


        ////FtcDashboard.getInstance().startCameraStream(camera, 0);


        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(320, 176, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addData("Camera Init Error", errorCode);
                telemetry.update();

            }
        });

        pipeline.changeMode(CalibrationPipeline.Mode.BLUECONE);

        telemetry.addData("Pipeline Mode", "Blue Cone");

        while (opModeInInit()) {
            // Toggle between camera positions

            if (gamepad1.a && aFlag) {
                if (servoPos == 0) {
                    cameraServo.setPosition(Constants.poleV2);
                    pipeline.changeMode(CalibrationPipeline.Mode.POLE);
                    telemetry.addData("Pipeline Mode", "Pole");
                    servoPos = 1;
                } else if (servoPos == 1) {
                    cameraServo.setPosition(Constants.sleeveV2);
                    pipeline.changeMode(CalibrationPipeline.Mode.SLEEVE);
                    telemetry.addData("Pipeline Mode", "Sleeve");
                    servoPos = 2;
                } else if (servoPos == 2) {
                    cameraServo.setPosition(Constants.coneV2);
                    pipeline.changeMode(CalibrationPipeline.Mode.REDCONE);
                    telemetry.addData("Pipeline Mode", "Red Cone");
                    servoPos = 3;
                } else {
                    cameraServo.setPosition(Constants.coneV2);
                    pipeline.changeMode((CalibrationPipeline.Mode.BLUECONE));
                    telemetry.addData("Pipeline Mode", "Blue Cone");
                    servoPos = 0;
                }
                aFlag = false;
            }
            if (!gamepad1.a) {
                aFlag = true;
            }

            // Change returned mat

            if (gamepad1.b && bFlag) {
                pipeline.toggleReturnedMat();
                bFlag = false;
            }
            if (!gamepad1.b) {
                aFlag = true;
            }

            // Telemetry

            if (servoPos == 1) {
                colors = pipeline.getYCrCbColors();
                telemetry.addData("Y", colors.val[0]);
                telemetry.addData("Cr", colors.val[1]);
                telemetry.addData("Cb", colors.val[2]);

                recYUpper = colors.val[0] + yRange;
                recYLower = colors.val[0] - yRange;

                recCrUpper = colors.val[1] + crRange;
                recCrLower = colors.val[1] - crRange;

                recCbUpper = colors.val[2] + cbRange;
                recCbLower = colors.val[2] - cbRange;

                telemetry.addData("Recommended Y Upper", colors.val[0] + yRange);
                telemetry.addData("Recommended Y Lower", colors.val[0] - yRange);

                telemetry.addData("Recommended Cr Upper", colors.val[1] + crRange);
                telemetry.addData("Recommended Cr Lower", colors.val[1] - crRange);

                telemetry.addData("Recommended Cb Upper", colors.val[2] + cbRange);
                telemetry.addData("Recommended Cb Lower", colors.val[2] - cbRange);
            } else {
                colors = pipeline.getHSVColors();
                telemetry.addData("H", colors.val[0]);
                telemetry.addData("S", colors.val[1]);
                telemetry.addData("V", colors.val[2]);

                recHUpper = colors.val[0] + hRange;
                recHLower = colors.val[0] - hRange;

                recSUpper = colors.val[1] + sRange;
                recSLower = colors.val[1] - sRange;

                recVUpper = colors.val[2] + vRange;
                recVLower = colors.val[2] - vRange;

                telemetry.addData("Recommended H Upper", colors.val[0] + hRange);
                telemetry.addData("Recommended H Lower", colors.val[0] - hRange);

                telemetry.addData("Recommended S Upper", colors.val[1] + sRange);
                telemetry.addData("Recommended S Lower", colors.val[1] - sRange);

                telemetry.addData("Recommended V Upper", colors.val[2] + vRange);
                telemetry.addData("Recommended V Lower", colors.val[2] - vRange);
            }

            // set pipeline values to recommended

            if (gamepad1.x) {
                if (servoPos == 0) {
                    pipeline.setBlueConeValues(recHLower, recSLower, recVLower, recHUpper, recSUpper, recVUpper);
                } else if (servoPos == 1) {
                    pipeline.setPoleValues(recHLower, recSLower, recVLower, recHUpper, recSUpper, recVUpper);
                } else if (servoPos == 2) {
                    pipeline.setSignalValues(recYLower, recCrLower, recCbLower, recYUpper, recCrUpper, recCbUpper);
                } else {
                    pipeline.setRedConeValues(recHLower, recSLower, recVLower, recHUpper, recSUpper, recVUpper);
                }
            }


        }

        waitForStart();


    }
}
