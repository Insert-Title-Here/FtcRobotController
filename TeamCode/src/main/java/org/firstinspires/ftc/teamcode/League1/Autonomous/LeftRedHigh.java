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

//TODO: Mirror
@Autonomous
public class LeftRedHigh extends LinearOpMode {
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


        color.setGain(300);
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
                        score.setLinkagePositionLogistic(Constants.linkageDown, 1000, 50);
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

        cameraServo.setPosition(0.5);


        while(opModeInInit()){
            telemetry.addData("red", color.getNormalizedColors().red);
            telemetry.addData("blue", color.getNormalizedColors().blue);
            telemetry.update();

        }


        waitForStart();
        parkPos = pipeline.getPosition();

        pipeline.setMode(false);

        armThread.start();
        feedForward.start();

        drive.addToLoggingString("originalColorRed: " + color.getNormalizedColors().red);
        drive.addToLoggingString("originalColorBlue: " + color.getNormalizedColors().blue);
        drive.addToLoggingString("");


        cameraServo.setPosition(0.73);



        drive.simpleMoveToPosition(-1600, MecDrive.MovementType.STRAIGHT, 0.35);
        //tankRotate(Math.PI / 4.25, 0.3);

        //TODO: figure out encoder val for this rotate
        drive.simpleMoveToPosition(230, MecDrive.MovementType.ROTATE, 0.4);
        //pipeline.normalizeToPole(0.3, 165, 5);
        //pipeline.Ynormalize(0.2, 95, 5);


        armUp.set(true);

        drive.simpleMoveToPosition(20, MecDrive.MovementType.STRAIGHT, 0.3);


        while(armUp.get()){

        }
        sleep(500);
        score.setGrabberPosition(0.7);
        sleep(500);
        score.setGrabberPosition(constants.grabbing);



        armDown.set(true);



        //drive.simpleMoveToPosition(140, MecDrive.MovementType.STRAIGHT, 0.3);

        //tankRotate(Math.PI / 2, 0.3);

        //TODO: figure out this rotate
        drive.simpleMoveToPosition(370, MecDrive.MovementType.ROTATE, 0.4);
        //pipeline.normalizeToPole(0.3, 82, 10);

        score.setGrabberPosition(0.85);







        //Dont know if need to check multiple time
        while(color.getNormalizedColors().red < 0.38 && color.getNormalizedColors().blue < 0.8){

            drive.setPowerAuto(0.4, MecDrive.MovementType.RDIAGONALLESS);
            telemetry.addData("blue", color.getNormalizedColors().blue);
            telemetry.addData("red", color.getNormalizedColors().red);
            telemetry.update();
        }

        drive.simpleBrake();






        drive.simpleMoveToPosition(-40, MecDrive.MovementType.STRAFE, 0.3);



        //score.setGrabberPosition(0.7);

        for(int i = 0; i < 3; i++) {


            //TODO: Logic doesnt work
            if(i != 0){


                while(color.getNormalizedColors().red < 0.23){

                    drive.setPowerAuto(0.4, MecDrive.MovementType.RDIAGONAL);
                    drive.addToLoggingString("ColorRed: " + color.getNormalizedColors().red);
                    drive.addToLoggingString("ColorBlue: " + color.getNormalizedColors().blue);
                    drive.addToLoggingString("");


                    telemetry.addData("blue", color.getNormalizedColors().blue);
                    telemetry.addData("red", color.getNormalizedColors().red);
                    telemetry.update();
                }


                drive.simpleBrake();



                drive.simpleMoveToPosition(60, MecDrive.MovementType.STRAFE, 0.3);


            }



            score.setLinkagePosition(0.73 + (i * 0.03));


            while (distance.getDistance(DistanceUnit.CM) > 4.3) {
                drive.setPowerAuto(0.35, MecDrive.MovementType.STRAIGHT);

                telemetry.addData("distance", distance.getDistance(DistanceUnit.CM));
                telemetry.update();

            }

            drive.simpleBrake();


            score.setGrabberPosition(constants.grabbing);
            //sleep(600);


            score.moveToPosition(200, 1);
            hold.set(true);
            sleep(300);

            drive.simpleMoveToPosition(-650, MecDrive.MovementType.STRAIGHT, 0.5);
            score.setLinkagePosition(Constants.linkageUp);



            //tankRotate(Math.PI / 4.35, 0.3);
            drive.simpleMoveToPosition(-320, MecDrive.MovementType.ROTATE, 0.4);
            //pipeline.normalizeToPole(0.3, 165, 10);
            //pipeline.Ynormalize(0.2, 92, 5);



            //armUp.set(true);

            drive.simpleMoveToPosition(-50, MecDrive.MovementType.STRAIGHT, 0.3);


            while(armUp.get()){

            }
            sleep(500);
            score.setGrabberPosition(0.7);
            sleep(500);

            armDown.set(true);



            //drive.simpleMoveToPosition(70, MecDrive.MovementType.STRAIGHT, 0.4);

            //tankRotate(Math.PI / 2, 0.3);
            drive.simpleMoveToPosition(320, MecDrive.MovementType.ROTATE, 0.4);
            //pipeline.normalizeToPole(0.3, 42, 5);

        }

        score.setGrabberPosition(constants.grabbing);

        camera.closeCameraDevice();

        drive.addToLoggingString("endColorRed: " + color.getNormalizedColors().red);
        drive.addToLoggingString("endColorBlue: " + color.getNormalizedColors().blue);
        drive.addToLoggingString("");



        if(parkPos == KevinGodPipeline.ParkPos.LEFT){
            drive.simpleMoveToPosition(-650, MecDrive.MovementType.STRAIGHT, 0.4);

        }else if(parkPos == KevinGodPipeline.ParkPos.RIGHT){
            drive.simpleMoveToPosition(650, MecDrive.MovementType.STRAIGHT, 0.4);

        }

        drive.writeLoggerToFile();

        //Will have to check if this aligns straight already (need color sensor or not) ->
        // may need to turn into slight diagonal instead of straight to check color
        //drive.simpleMoveToPosition(675, MecDrive.MovementType.STRAIGHT, 0.3);


    }


    //TODO: Test if we want to use IMU again
    public void tankRotate(double radians, double power){

        if(radians > imu.getAngularOrientation().firstAngle){
            power *= -1;
        }

        if(power > 0) {
            while (Math.abs(imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle) < Math.abs(radians)) {
                telemetry.addData("target", radians);
                telemetry.addData("current", imu.getAngularOrientation().firstAngle);
                telemetry.update();
                drive.setPowerAuto(power, MecDrive.MovementType.ROTATE);
            }
        }else {
            while (Math.abs(imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle) > Math.abs(radians)) {
                telemetry.addData("target", radians);
                telemetry.addData("current", imu.getAngularOrientation().firstAngle);
                telemetry.update();
                drive.setPowerAuto(power, MecDrive.MovementType.ROTATE);
            }
        }

        drive.simpleBrake();




    }

    public void normalizeToPole(double power, int xMin, int xMax) {
        while(pipeline.getPolePosition() > xMax || pipeline.getPolePosition() < xMin) {
            if(pipeline.getPolePosition() > xMax) {
                drive.setPowerAuto(power, MecDrive.MovementType.ROTATE);
            } else {
                drive.setPowerAuto(-power, MecDrive.MovementType.ROTATE);
            }
        }
    }
}