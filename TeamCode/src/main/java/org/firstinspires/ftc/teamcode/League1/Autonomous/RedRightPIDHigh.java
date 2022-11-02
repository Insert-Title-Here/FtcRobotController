package org.firstinspires.ftc.teamcode.League1.Autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.League1.Autonomous.Vision.KevinGodPipeline;
import org.firstinspires.ftc.teamcode.League1.Common.Constants;
import org.firstinspires.ftc.teamcode.League1.Subsystems.MecDrive;
import org.firstinspires.ftc.teamcode.League1.Subsystems.ScoringSystem2;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.concurrent.atomic.AtomicBoolean;

@Autonomous
public class RedRightPIDHigh extends LinearOpMode {
    MecDrive drive;
    ScoringSystem2 score;
    Constants constants;
    Thread armThread, feedForward, idController;
    PIDCoefficients pid = new PIDCoefficients(0, 0, 0);
    AtomicBoolean hold, armUp, armDown;

    BNO055IMU imu;
    ColorRangeSensor distance, color;
    Servo cameraServo;

    OpenCvWebcam camera;
    KevinGodPipeline pipeline;
    KevinGodPipeline.ParkPos parkPos;

    int normalizeDistance;




    @Override
    public void runOpMode() throws InterruptedException {
        drive = new MecDrive(hardwareMap, false, telemetry);
        constants = new Constants();
        score = new ScoringSystem2(hardwareMap, constants);
        hold = new AtomicBoolean(false);
        armUp = new AtomicBoolean(false);
        armDown = new AtomicBoolean(false);

        score.setLinkagePosition(Constants.linkageDown);
        score.setGrabberPosition(constants.grabbing);

        distance = hardwareMap.get(ColorRangeSensor.class, "distance");
        color = hardwareMap.get(ColorRangeSensor.class, "color");
        cameraServo = hardwareMap.get(Servo.class, "camera");


        color.setGain(600);
        distance.setGain(300);


        /*
        idController = new Thread(){
            @Override
            public void run() {



                while(opModeIsActive()){
                    if(hold.get()){
                        ElapsedTime time = new ElapsedTime();
                        double startTime = time.milliseconds();

                        int leftIntegralSum = 0;
                        int rightIntegralSum = 0;


                        int rLiftPos = score.getRightEncoderPos();
                        int lLiftPos = -1 * score.getLeftEncoderPos();

                        int tics = score.getHeight();

                        int leftPreviousError = Math.abs(tics - lLiftPos);
                        int rightPreviousError = Math.abs(tics - rLiftPos);

                        while(hold.get()){

                            rLiftPos = score.getRightEncoderPos();
                            lLiftPos = -1 * score.getLeftEncoderPos();

                            double currentTime = time.milliseconds();

                            int leftError = tics - lLiftPos;
                            int rightError = tics - rLiftPos;

                            leftIntegralSum += (0.5 * (leftError + leftPreviousError) * (currentTime - startTime));
                            rightIntegralSum += (0.5 * (rightError + rightPreviousError) * (currentTime - startTime));

                            double leftDerivative = (leftError - leftPreviousError)/(currentTime - startTime);
                            double rightDerivative = (rightError - rightPreviousError)/(currentTime - startTime);

                            double leftPower = (pid.i * leftIntegralSum) + (pid.d * leftDerivative);
                            double rightPower = (pid.i * rightIntegralSum) + (pid.d * rightDerivative);

                            if(tics < ((rLiftPos + lLiftPos) / 2)){
                                leftPower *= -1;
                                rightPower *= -1;
                            }

                            score.setPower(rightPower, leftPower);


                            startTime = currentTime;
                            leftPreviousError = leftError;
                            rightPreviousError = rightError;



                        }




                    }
                }
            }
        };

         */

        armThread = new Thread(){
            @Override
            public void run() {
                //score.setLinkagePosition(0.7);
                while(opModeIsActive()) {
                    if(armUp.get()) {
                        hold.set(false);
                        score.moveToPosition(830, 1);
                        hold.set(true);
                        score.setLinkagePositionLogistic(Constants.linkageScore, 500, 50);
                        armUp.set(false);
                    }else if(armDown.get()){
                        hold.set(false);
                        score.setLinkagePosition(Constants.linkageUp);
                        score.moveToPosition(0, 0.5);
                        score.setLinkagePositionLogistic(Constants.linkageDown, 250, 30);
                        armDown.set(false);
                    }




                }

                //Might need this
                //hold.set(true);
            }
        };



        feedForward = new Thread(){
            @Override
            public void run() {
                while(opModeIsActive()){
                    if(hold.get()){
                        score.setPower(0.2);
                    }
                }
            }
        };






        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.RADIANS;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";

        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);



        WebcamName webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);
        pipeline = new KevinGodPipeline(telemetry, drive);

        camera.setPipeline(pipeline);


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

        FtcDashboard.getInstance().startCameraStream(camera, 0);

        pipeline.setMode(false);




        cameraServo.setPosition(0.5);






        waitForStart();
        parkPos = pipeline.getPosition();



        armThread.start();
        feedForward.start();

        drive.addToLoggingString("originalColorRed: " + color.getNormalizedColors().red);
        drive.addToLoggingString("originalColorBlue: " + color.getNormalizedColors().blue);
        drive.addToLoggingString("");


        cameraServo.setPosition(0.73);



        drive.goTOPIDPos(-2000, 0.7,MecDrive.MovementType.STRAIGHT);
        drive.tankRotatePID(Math.PI / 4, 1);

        //drive.simpleMoveToPosition(-250, MecDrive.MovementType.ROTATE, 0.4);
        normalizeDistance = pipeline.normalizeToPole(0.3, 165, 5);
        //pipeline.Ynormalize(0.2, 95, 5);


        armUp.set(true);

        //drive.simpleMoveToPosition(30, MecDrive.MovementType.STRAIGHT, 0.3);
        while(armUp.get()){

        }
        sleep(500);
        score.setGrabberPosition(0.3);
        sleep(500);
        score.setGrabberPosition(constants.grabbing);

        armDown.set(true);

        //drive.simpleMoveToPosition(140, MecDrive.MovementType.STRAIGHT, 0.3);

        drive.tankRotatePID(Math.PI / 2, 1);
        //drive.simpleMoveToPosition(-370 - normalizeDistance, MecDrive.MovementType.ROTATE, 0.4);
        //pipeline.normalizeToPole(0.3, 82, 10);

        score.setGrabberPosition(0.7);


        //drive.tankRotatePID(Math.PI/2, 1);            //pipeline.normalizeToPole(0.3, 42, 5);

        drive.simpleMoveToPosition(120, MecDrive.MovementType.STRAFE, 0.4);
        drive.goTOPIDPos(120, 1, MecDrive.MovementType.STRAIGHT);






        //Dont know if need to check multiple time
        drive.autoDiagonals(false);

        drive.simpleBrake();






        drive.simpleMoveToPosition(-40, MecDrive.MovementType.STRAFE, 0.3);


        for(int i = 0; i < 2; i++) {


            //TODO: Logic doesnt work
            if(i != 0){
                //score.setLinkagePosition(0.12);


                drive.autoDiagonals(false);



                drive.simpleBrake();



                drive.simpleMoveToPosition(-40, MecDrive.MovementType.STRAFE, 0.3);


            }



            score.setLinkagePosition(0.795 + (i * 0.03));


            while (distance.getDistance(DistanceUnit.CM) > 4.3) {
                drive.setPowerAuto(0.3, MecDrive.MovementType.STRAIGHT);

                telemetry.addData("distance", distance.getDistance(DistanceUnit.CM));
                telemetry.update();

            }
            drive.simpleBrake();


            score.setGrabberPosition(constants.grabbing);
            sleep(600);

            score.moveToPosition(200, 1);
            hold.set(true);
            sleep(300);

            drive.goTOPIDPos(-650,  0.5, MecDrive.MovementType.STRAIGHT);
            score.setLinkagePosition(Constants.linkageUp);

            drive.tankRotatePID(Math.PI / 4, 1);
            //drive.simpleMoveToPosition(290, MecDrive.MovementType.ROTATE, 0.4);
            normalizeDistance = pipeline.normalizeToPole(0.2, 172, 3);



            armUp.set(true);

            drive.simpleMoveToPosition(-80, MecDrive.MovementType.STRAIGHT, 0.3);

            while(armUp.get()){

            }
            sleep(500);
            score.setGrabberPosition(0.3);
            sleep(500);

            armDown.set(true);

            //drive.simpleMoveToPosition(70, MecDrive.MovementType.STRAIGHT, 0.4);

            drive.tankRotatePID(Math.PI / 2, 1);
            //drive.simpleMoveToPosition(-320 + normalizeDistance, MecDrive.MovementType.ROTATE, 0.4);
            drive.tankRotatePID(Math.PI/2, 1);            //pipeline.normalizeToPole(0.3, 42, 5);

            if(i != 1) {
                drive.simpleMoveToPosition(150, MecDrive.MovementType.STRAFE, 0.4);
                drive.goTOPIDPos(150, 1, MecDrive.MovementType.STRAIGHT);

                score.setGrabberPosition(0.7);
            }

        }

        score.setGrabberPosition(constants.grabbing);

        camera.closeCameraDevice();

        drive.addToLoggingString("endColorRed: " + color.getNormalizedColors().red);
        drive.addToLoggingString("endColorBlue: " + color.getNormalizedColors().blue);
        drive.addToLoggingString("");


/*
        if(parkPos == KevinGodPipeline.ParkPos.LEFT){
            drive.simpleMoveToPosition(-650, MecDrive.MovementType.STRAIGHT, 1);

        }else if(parkPos == KevinGodPipeline.ParkPos.RIGHT){
            drive.simpleMoveToPosition(650, MecDrive.MovementType.STRAIGHT, 1);

        }


 */
        drive.writeLoggerToFile();

        //Will have to check if this aligns straight already (need color sensor or not) ->
        // may need to turn into slight diagonal instead of straight to check color
        //drive.simpleMoveToPosition(675, MecDrive.MovementType.STRAIGHT, 0.3);


    }



}