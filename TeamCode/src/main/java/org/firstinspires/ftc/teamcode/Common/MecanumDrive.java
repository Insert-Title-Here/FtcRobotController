package org.firstinspires.ftc.teamcode.Common;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;
import java.io.FileNotFoundException;
import java.io.PrintStream;
import java.util.concurrent.atomic.AtomicBoolean;
//@Config
public class MecanumDrive {
    DcMotor fl, fr, bl, br;
    Telemetry telemetry;
    Thread driveThread;
    AtomicBoolean active;
    BNO055IMU imu;
    ColorRangeSensor colorTape;


   //OpenCvWebcam webcam;
    //ContourMultiScore detect;

    private double accumulatedError;
    private double error;

    public static double proportionCoefficient = 0.001;//0.75
    public static double integralCoefficient = 0;
    public static double derivativeCoefficient = 1.2;//0.8

    // creates/accesses file
    File loggingFile = AppUtil.getInstance().getSettingsFile("driveTelemetry.txt");
    // holds data
    public String loggingString;
    //constructor
    public MecanumDrive(HardwareMap hardwareMap, Telemetry telemetry) {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // seehe calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";

        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        this.telemetry = telemetry;
        active = new AtomicBoolean();
        colorTape = hardwareMap.get(ColorRangeSensor.class, "colorTape");
        accumulatedError = 0;



//        detect = new ContourMultiScore(telemetry);
//        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
//        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
//        webcam.setPipeline(detect);
//
//        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
//
//            @Override
//            public void onOpened() {
//                webcam.startStreaming(320, 176, OpenCvCameraRotation.UPRIGHT);
//            }
//
//            @Override
//            public void onError(int errorCode) {
//                telemetry.addData("Camera Init Error", errorCode);
//                telemetry.update();
//            }
//        });



        //initiallises drive motors
        fl = hardwareMap.get(DcMotor.class, "fl");
        fr = hardwareMap.get(DcMotor.class, "fr");
        bl = hardwareMap.get(DcMotor.class, "bl");
        br = hardwareMap.get(DcMotor.class, "br");

        fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        br.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        fl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        fl.setDirection(DcMotor.Direction.REVERSE);
        fr.setDirection(DcMotor.Direction.FORWARD);
        bl.setDirection(DcMotor.Direction.REVERSE);
        br.setDirection(DcMotor.Direction.FORWARD);

//        // ftc dashboard
//        FtcDashboard.getInstance().startCameraStream(webcam, 0);


    }
    public void resetIMU(){
        //initializes imu
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // seehe calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";

        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu.initialize(parameters);
    }
    /*current motor positions*/
    public int getFLPosition() {
        return fl.getCurrentPosition();
    }

    public int getFRPosition() {
        return fr.getCurrentPosition();

    }

    public int getBLPosition() {
        return bl.getCurrentPosition();

    }

    public int getBRPosition() {
        return br.getCurrentPosition();

    }


    public void setPower(Vector2D velocity, double turnValue, boolean isSwapped) {
        turnValue = -turnValue;
        double direction = velocity.getDirection();


        double power = velocity.magnitude();

        double angle = direction + 3 * Math.PI / 4.0;
        double sin = Math.sin(angle);
        double cos = Math.cos(angle);

        if (!isSwapped) {
            setPower((power * sin - turnValue), (power * cos + turnValue),
                    (power * cos - turnValue), (power * sin + turnValue));
        } else {
            setPower(-(power * sin - turnValue), -(power * cos + turnValue),
                    -(power * cos - turnValue), -(power * sin + turnValue));
        }
    }


    public void setPower(double flPow, double frPow, double blPow, double brPow) {
        fl.setPower(flPow);
        fr.setPower(frPow);
        bl.setPower(blPow);
        br.setPower(brPow);
    }

    public void goToPosition(double flPow, double frPow, double blPow, double brPow, int tics, String action) {
        //fl fr bl br

        fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        br.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        fl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // won't work for turns, only forward and backward
        //avg position from all four drive motors
        int position = (int) (Math.abs(fl.getCurrentPosition()) + Math.abs(fr.getCurrentPosition()) /*+ Math.abs(bl.getCurrentPosition())*/ +
                Math.abs(br.getCurrentPosition())) / 3;

        telemetry.addData("motorPosition", position);
        telemetry.update();
        long time = System.currentTimeMillis();
        long difference;
        //Encoder based gotoposition
        while ((Math.abs(tics) - position) > 0) {
            difference = System.currentTimeMillis() - time;
            setPower(flPow, frPow, blPow, brPow);
            position = (int) (Math.abs(fl.getCurrentPosition()) + Math.abs(fr.getCurrentPosition()) /*+ Math.abs(bl.getCurrentPosition()) */+
                    Math.abs(br.getCurrentPosition())) / 3;
        }

        setPower(0, 0, 0, 0);

    }

    //TODO: Consider to do PID for each individual wheel
    public void goToPositionPID(int tics, String action) {
        //fl fr bl br

        fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        br.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        fl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // won't work for turns, only forward and backward
        //avg position from all four drive motors
        //int position = (int) (Math.abs(fl.getCurrentPosition()) + Math.abs(fr.getCurrentPosition()) + Math.abs(bl.getCurrentPosition()) +
                //Math.abs(br.getCurrentPosition())) / 4;
        int position = (int)(Math.abs(fr.getCurrentPosition()));

        telemetry.addData("motorPosition", position);
        telemetry.update();
        long time = System.currentTimeMillis();
        long timeDifference = 0;
        double priorError = tics;
        double currentError = tics;
        double priorAngleError = 0;
        double currentAngleError = 0;
        //Encoder based gotoposition
        //Math.abs(currentError) > 0.001
        while ((Math.abs(tics) - position) > 0) {
            double drivePower = 0;
            double addedDrivePow = 0;
            //This below is PID for each individual wheel
            //setPower(PIDfl(priorError, currentError, timeDifference), PIDfr(priorError, currentError, timeDifference), PIDbl(priorError, currentError, timeDifference), PIDbr(priorError, currentError, timeDifference));
            drivePower = PIDDrivePower(priorError, currentError, timeDifference);
            addedDrivePow = additionalPow(priorAngleError, currentAngleError, timeDifference);
            priorAngleError = currentAngleError;
            priorError = currentError;
            /*
            if(imu.getAngularOrientation().firstAngle > 0){
                setPower(drivePower, drivePower - addedDrivePow, drivePower, drivePower - addedDrivePow);
            }else if(imu.getAngularOrientation().firstAngle < 0){
                setPower(drivePower - addedDrivePow, drivePower, drivePower - addedDrivePow, drivePower);
            }else{

             */
                setPower(drivePower, drivePower, drivePower, drivePower);
            //}
            //position = (int) (Math.abs(fl.getCurrentPosition()) + Math.abs(fr.getCurrentPosition()) /*+ Math.abs(bl.getCurrentPosition())*/ +
                    //Math.abs(br.getCurrentPosition())) / 3;
            position = (int)Math.abs(fr.getCurrentPosition());
            currentError = tics - position;
            currentAngleError = Math.abs(imu.getAngularOrientation().firstAngle);
            timeDifference = System.currentTimeMillis() - time;
            accumulateError(timeDifference, currentError);
            if(timeDifference > tics * 1)break;
        }
        accumulatedError = 0;
        loggingString +="----------------------------------------------------------------------------\n";
        /*
        loggingString += action.toUpperCase() + "\n";
        loggingString += "FL Position: " + getFLPosition() + "\n";
        loggingString += "FR Position: " + getFRPosition() + "\n";
        loggingString += "BL Position: " + getBLPosition() + "\n";
        loggingString += "BR Position: " + getBRPosition() + "\n";
        loggingString += "---------------------" + "\n";

         */

        // loggingString += "Claw (Intake) Position: " + score.getClawPosition()

        setPower(0, 0, 0, 0);

    }
    public void goToPositionPIDUltra(int tics) {
        //fl fr bl br

        fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        br.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        fl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        // Marcus Lam waas here...
        bl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // won't work for turns, only forward and backward
        //avg position from all four drive motors
        //int position = (int) (Math.abs(fl.getCurrentPosition()) + Math.abs(fr.getCurrentPosition()) + Math.abs(bl.getCurrentPosition()) +
        //Math.abs(br.getCurrentPosition())) / 4;
        int positionFl = (int)(Math.abs(fl.getCurrentPosition()));
        int positionBl = (int)(Math.abs(bl.getCurrentPosition()));
        int positionFr = (int)(Math.abs(fr.getCurrentPosition()));
        int positionBr = (int)(Math.abs(br.getCurrentPosition()));
        int position = 0;
        long time = System.currentTimeMillis();
        long timeDifference = 0;
        double priorError = tics;
        double currentError = tics;
        double priorAngleError = 0;
        double currentAngleError = 0;

        double priorErrorFl = tics;
        double priorErrorBl = tics;
        double priorErrorFr = tics;
        double priorErrorBr = tics;

        double currentErrorFl = 0;
        double currentErrorBl = 0;
        double currentErrorFr = 0;
        double currentErrorBr = 0;
        //Encoder based gotoposition
        //Math.abs(currentError) > 0.001
        while ((Math.abs(tics) - position) > 0) {
            double drivePower = 0;
            double addedDrivePow = 0;
            //This below is PID for each individual wheel
            //setPower(PIDfl(priorError, currentError, timeDifference), PIDfr(priorError, currentError, timeDifference), PIDbl(priorError, currentError, timeDifference), PIDbr(priorError, currentError, timeDifference));
            drivePower = PIDDrivePower(priorError, currentError, timeDifference);
            priorError = currentError;

            setPower(drivePower, drivePower, drivePower, drivePower);

            position = (int) (Math.abs(fl.getCurrentPosition()) + Math.abs(fr.getCurrentPosition()) + Math.abs(bl.getCurrentPosition()) +
            Math.abs(br.getCurrentPosition())) / 3;
            currentError = tics - position;
            currentAngleError = Math.abs(imu.getAngularOrientation().firstAngle);
            timeDifference = System.currentTimeMillis() - time;
            accumulateError(timeDifference, currentError);
            if(timeDifference > tics * 1)break;
        }

        // DDD

        accumulatedError = 0;
        loggingString +="----------------------------------------------------------------------------\n";
        /*
        loggingString += action.toUpperCase() + "\n";
        loggingString += "FL Position: " + getFLPosition() + "\n";
        loggingString += "FR Position: " + getFRPosition() + "\n";
        loggingString += "BL Position: " + getBLPosition() + "\n";
        loggingString += "BR Position: " + getBRPosition() + "\n";
        loggingString += "---------------------" + "\n";

         */

        // loggingString += "Claw (Intake) Position: " + score.getClawPosition()

        setPower(0, 0, 0, 0);

    }

    public void goToPosition(double flPow, double frPow, double blPow, double brPow) {
        //fl fr bl br

        fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        br.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        fl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // won't work for turns, only forward and backward

        int position = (int) (Math.abs(fl.getCurrentPosition()) + Math.abs(fr.getCurrentPosition()) /*+ Math.abs(bl.getCurrentPosition())*/ +
                Math.abs(br.getCurrentPosition())) / 3;

        telemetry.addData("motorPosition", position);
        telemetry.update();


        setPower(flPow, frPow, blPow, brPow);

    }
    // Turns a certain amount of given radians useing imu
    public void turn(double radians) {
        double integralPow;
        double derivativePow;
        double proportionPow;
        ElapsedTime time = new ElapsedTime();
        double initialAngle = imu.getAngularOrientation().firstAngle;
        double currentAngleError = angleWrap(radians);
        double priorAngleError = angleWrap(radians);
        double priorTime = time.milliseconds();
        double timeDifference = 0;
        double before;
        double after;
       // loggingString += "CurrentAngle: " + initialAngle + "\n";
        while (Math.abs((imu.getAngularOrientation().firstAngle - initialAngle)) < Math.abs(radians)) {
            double drivePower = 0;
            if(currentAngleError == priorAngleError){
                //values stay the same
            }else{
                priorAngleError = angleWrap(radians - (imu.getAngularOrientation().firstAngle - initialAngle));
            }
            accumulateError(timeDifference, currentAngleError);

            drivePower = PIDTurnPower(priorAngleError, currentAngleError, timeDifference);
            //turn left # of radians
            setPower(-drivePower, drivePower, -drivePower, drivePower);

            timeDifference = time.milliseconds() - priorTime;
            currentAngleError = angleWrap(radians - (imu.getAngularOrientation().firstAngle - initialAngle));
            if(time.milliseconds() - priorTime > 1300) break;

        }
        //resetIMU();
        error = 0;
    }
    // turns to the starting position
    //TODO: Needs Testing
    public void turnToInitialPosition(double radians) {
        boolean over = true;
        double rad = radians;
        if (rad > 0.04 || rad < -0.04) {
            while ((Math.abs(imu.getAngularOrientation().firstAngle)) > 0.005 && over) {
                double currentRadians = imu.getAngularOrientation().firstAngle;
                if (0 < currentRadians) {
                    //turn right # of radians
                    setPower(0.2, -0.2, 0.2, -0.2);
                } else if (currentRadians < 0) {
                    //turn left # of radians
                    setPower(-0.2, 0.2, -0.2, 0.2);
                } else {
                    over = false;
                }
            }
        }
    }
    /*
    //DO NOT USE currently does not work
    public void goToPositionTest(int flTics, int frTics, int blTics, int brTics, double power, String action) {
        //fl fr bl br
        active.set(true);
        fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        br.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        fl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        driveThread = new Thread() {
            @Override
            public void run() {
                boolean flDone = false;
                boolean frDone = false;
                boolean blDone = false;
                boolean brDone = false;

                while (active.get()) {
                    if (flDone && frDone && blDone && brDone) {
                        active.set(false);
                    } else {
                        //fl
                        if ((Math.abs(flTics) - Math.abs(fl.getCurrentPosition())) > 0) {
                            setPower(power, power, power, power);
                        } else {
                            flDone = true;
                        }
                        //fr
                        if ((Math.abs(frTics) - Math.abs(fr.getCurrentPosition())) > 0) {
                            setPower(power, power, power, power);
                        } else {
                            frDone = true;
                        }
                        //bl
                        if ((Math.abs(blTics) - Math.abs(bl.getCurrentPosition())) > 0) {
                            setPower(power, power, power, power);
                        } else {
                            blDone = true;
                        }
                        //br
                        if ((Math.abs(brTics) - Math.abs(br.getCurrentPosition())) > 0) {
                            setPower(power, power, power, power);
                        } else {
                            brDone = true;
                        }
                        telemetry.addData("flPos", getFLPosition());
                        telemetry.addData("frPos", getFRPosition());
                        telemetry.addData("blPos", getBLPosition());
                        telemetry.addData("brPos", getBRPosition());
                        telemetry.addData("red", currentRedColor());
                        telemetry.addData("blue", currentBlueColor());
                        telemetry.update();
                    }

                }
                setPower(0, 0, 0, 0);
            }
        };
        driveThread.start();


    }
    //TODO: do liftsystem PID
     */
    //PID testing not currently operational
    public double PIDTurnPower(double priorError, double currentError, double timeChange) {
        double proportionCoefficient = 0.845;//0.845
        double integralCoefficient = 0;
        double derivativeCoefficient = 0.5;//0.5
        error = currentError;
        /*
        loggingString += "PriorAngleError: " + priorError + "   ";
        loggingString += "CurrentAngleError: " + currentError + "   ";
        loggingString += "DrivePower: " +  currentError * proportionCoefficient + ((currentError-priorError)/timeChange) * derivativeCoefficient + getAccumulatedError() * integralCoefficient + "   ";
        loggingString += "derivativePower: " + ((currentError-priorError)/timeChange) * derivativeCoefficient + "   ";
        loggingString += "proportionPower: " + getAccumulatedError() * integralCoefficient + "   ";
        loggingString += "CurrentAngle: " + imu.getAngularOrientation().firstAngle + "\n";

         */
        return currentError * proportionCoefficient + ((currentError-priorError)/timeChange) * derivativeCoefficient + getAccumulatedError() * integralCoefficient;
    }
    //TODO
    public double PIDDrivePower(double priorError, double currentError, double timeChange){
        double proportionCoefficient = 0.35;//0.75
        double integralCoefficient =0 ;
        double derivativeCoefficient = 0.0011;//0.8
        double totalPower = currentError * proportionCoefficient + ((currentError-priorError)/timeChange) * derivativeCoefficient + getAccumulatedError() * integralCoefficient;
        loggingString += "PriorError: " + priorError + "   ";
        loggingString += "CurrentError: " + currentError + "   ";
        loggingString += "derivativePower: " + ((currentError-priorError)/timeChange) * derivativeCoefficient + "   ";
        loggingString += "proportionPower: " + currentError * proportionCoefficient+ "   ";
        loggingString += "integralPower: " + getAccumulatedError() * integralCoefficient + "   ";
        loggingString += "DrivePower: " + totalPower + "\n";
        error = currentError;
        return totalPower;
    }
    public double PIDfl(double priorError, double currentError, double timeChange){
        double proportionCoefficient = 0.001;//0.75
        double integralCoefficient = 0;
        double derivativeCoefficient = 1.2;//0.8
        double totalPower = currentError * proportionCoefficient + ((currentError-priorError)/timeChange) * derivativeCoefficient + getAccumulatedError() * integralCoefficient;
        error = currentError;
        return totalPower;
    }

    public double PIDbl(double priorError, double currentError, double timeChange){
        double proportionCoefficient = 0.001;//0.75
        double integralCoefficient = 0;
        double derivativeCoefficient = 1.2;//0.8
        double totalPower = currentError * proportionCoefficient + ((currentError-priorError)/timeChange) * derivativeCoefficient + getAccumulatedError() * integralCoefficient;
        error = currentError;
        return totalPower;
    }
    public double PIDfr(double priorError, double currentError, double timeChange){
        double proportionCoefficient = 0.001;//0.75
        double integralCoefficient = 0;
        double derivativeCoefficient = 1.2;//0.8
        double totalPower = currentError * proportionCoefficient + ((currentError-priorError)/timeChange) * derivativeCoefficient + getAccumulatedError() * integralCoefficient;
        error = currentError;
        return totalPower;
    }
    public double PIDbr(double priorError, double currentError, double timeChange){
        double proportionCoefficient = 0.001;//0.75
        double integralCoefficient = 0;
        double derivativeCoefficient = 1.2;//0.8
        double totalPower = currentError * proportionCoefficient + ((currentError-priorError)/timeChange) * derivativeCoefficient + getAccumulatedError() * integralCoefficient;
        error = currentError;
        return totalPower;
    }
    //TODO
    //used for robot not moving straight
    public double additionalPow(double priorError, double currentError, double timeChange){
        double proportionCoefficient = 0.1;
        double integralCoefficient = 0;
        double derivativeCoefficient = 0.06;
        return currentError * proportionCoefficient + ((currentError-priorError)/timeChange) * derivativeCoefficient + getAccumulatedError() * integralCoefficient;

    }
    public double getError(){
        return error;
    }
    public void accumulateError(double delta_time, double error){
        accumulatedError += error * delta_time;
    }
    public double getAccumulatedError(){
        return accumulatedError;
    }
    // resets encoders
    public void resetEncoders() {
        fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        br.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        fl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    // returns the power of a mecanum wheel
    public double getPower(){
        return fl.getPower();
    }




    // logs string into file
    public void writeLoggerToFile(){
        try{
            PrintStream toFile = new PrintStream(loggingFile);
            toFile.println(loggingString);
        }catch(FileNotFoundException e){
            e.printStackTrace();
        }
    }
    // for color sensor, returns the blue color
    public int currentBlueColor() {
        return colorTape.blue(); // if current color is really high // 410
    }
    // for color sensor, returns the blue colo
    public int currentRedColor() {
        return colorTape.red(); // if current color is really high // 177
    }
    boolean temp = true;
    //checks if the color sensor identifies tape color
    public void findTape(String color) {
        if (color.equalsIgnoreCase("blue")) {
            while (currentBlueColor() < 400) { //blue tape
                if (temp) {
                    goToPosition(0.2, 0.4, 0.4, 0.2);
                    // strafe diagonal left
                    //goToPosition(0.4, 0, 0, 0.4);
                    temp = false;
                }

            }
        } else if (color.equalsIgnoreCase("red")) {
            while (currentRedColor() < 195) { //red tape
                if (temp) {
                    goToPosition(0.4, 0.2, 0.2, 0.4);
                    // strafe diagonal left
                    //goToPosition(0.4, 0, 0, 0.4);
                    temp = false;
                }

            }
        }
        goToPosition(0,0,0,0);
    }
    public double angleWrap(double radians){
        while(radians > Math.PI){
            radians -= 2 * Math.PI;
        }
        while(radians < -Math.PI){
            radians += 2 * Math.PI;
        }
        return radians;
    }
    // returns the average tics for mecanum wheels
    public int avgPosition(){
        return (int)(Math.abs(fl.getCurrentPosition()) + Math.abs(fr.getCurrentPosition()) + Math.abs(bl.getCurrentPosition()) + Math.abs(br.getCurrentPosition()))/4;
    }
    // returns the average tics for mecanum wheels (inputed)
    public int avgPosition(int fl, int fr, int bl, int br){
        return (int)(Math.abs(fl) + Math.abs(fr) + Math.abs(bl) + Math.abs(br))/4;
    }
}
