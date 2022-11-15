package org.firstinspires.ftc.teamcode.Common;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;
import java.io.FileNotFoundException;
import java.io.PrintStream;
import java.util.concurrent.atomic.AtomicBoolean;

public class MecanumDrive {
    DcMotor fl, fr, bl, br;
    Telemetry telemetry;
    Thread driveThread;
    AtomicBoolean active;
    BNO055IMU imu;
    ColorRangeSensor colorTape;
    double accumulatedError;
    double integralPow;
    double derivativePow;
    double proportionPow;
    double angleError;
    // creates/accesses file
    File loggingFile = AppUtil.getInstance().getSettingsFile("telemetry.txt");
    // holds data
    public String loggingString;
    //constructor
    public MecanumDrive(HardwareMap hardwareMap, Telemetry telemetry) {
        //initializes imu
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

        fl.setDirection(DcMotorSimple.Direction.REVERSE);
        fr.setDirection(DcMotorSimple.Direction.FORWARD);
        bl.setDirection(DcMotorSimple.Direction.REVERSE);
        br.setDirection(DcMotorSimple.Direction.FORWARD);

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
        int position = (int) (Math.abs(fl.getCurrentPosition()) + Math.abs(fr.getCurrentPosition()) + Math.abs(bl.getCurrentPosition()) +
                Math.abs(br.getCurrentPosition())) / 4;

        telemetry.addData("motorPosition", position);
        telemetry.update();
        long time = System.currentTimeMillis();
        long timeDifference;
        //Encoder based gotoposition
        while ((Math.abs(tics) - position) > 0) {
            timeDifference = System.currentTimeMillis() - time;

            setPower(flPow, frPow, blPow, brPow);
            position = (int) (Math.abs(fl.getCurrentPosition()) + Math.abs(fr.getCurrentPosition()) + Math.abs(bl.getCurrentPosition()) +
                    Math.abs(br.getCurrentPosition())) / 4;
        }
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

        int position = (int) (Math.abs(fl.getCurrentPosition()) + Math.abs(fr.getCurrentPosition()) + Math.abs(bl.getCurrentPosition()) +
                Math.abs(br.getCurrentPosition())) / 4;

        telemetry.addData("motorPosition", position);
        telemetry.update();


        setPower(flPow, frPow, blPow, brPow);

    }
    // Turns a certain amount of given radians useing imu
    //TODO: Needs Testing
    public void turn(double radians, double power) {
        ElapsedTime time = new ElapsedTime();
        double initialAngle = imu.getAngularOrientation().firstAngle;
        double currentAngleError = angleWrap(radians);
        double priorAngleError = angleWrap(radians);
        double priorTime = time.milliseconds();
        double timeDifference = 0;
        double before;
        double after;
        loggingString += "CurrentAngle: " + initialAngle + "\n";
        while (Math.abs((imu.getAngularOrientation().firstAngle - initialAngle)) < Math.abs(radians)) {
            double drivePower = 0;
            if(currentAngleError == priorAngleError){
                //values stay the same
            }else{
                priorAngleError = angleWrap(radians - (imu.getAngularOrientation().firstAngle - initialAngle));
            }
            accumulateError(timeDifference, currentAngleError);
            if (radians < 0) {
                drivePower = additionalPower(priorAngleError, currentAngleError, timeDifference);
                //turn right # of radians
                setPower(drivePower, -drivePower, drivePower, -drivePower);
            } else {
                drivePower = additionalPower(priorAngleError, currentAngleError, timeDifference);
                //turn left # of radians
                setPower(-drivePower, drivePower, -drivePower, drivePower);
            }
            timeDifference = time.milliseconds() - priorTime;
            currentAngleError = angleWrap(radians - (imu.getAngularOrientation().firstAngle - initialAngle));


        }
        integralPow = 0;
        derivativePow = 0;
        proportionPow = 0;
        angleError = 0;


    }
    // turns to the starting position
    //TODO: Needs Testing
    public void turnToInitialPosition() {
        boolean over = true;
        double rad = imu.getAngularOrientation().firstAngle;
        if (rad > 0.04 || rad < -0.04) {
            while ((Math.abs(imu.getAngularOrientation().firstAngle)) > 0.005 && over) {
                double radians = imu.getAngularOrientation().firstAngle;
                if (0 < radians) {
                    //turn right # of radians
                    setPower(0.2, -0.2, 0.2, -0.2);
                } else if (radians < 0) {
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

     */
    //PID testing not currently operational
    public double additionalPower(double priorError, double currentError, double timeChange) {
        double proportionCoefficient = 0.725;//0.75
        double integralCoefficient = 0;
        double derivativeCoefficient = /*0.007*/0;
        angleError = currentError;
        integralPow = getAccumulatedError() * integralCoefficient;
        derivativePow = ((currentError-priorError)/timeChange) * derivativeCoefficient;
        proportionPow = currentError * proportionCoefficient;
        loggingString += "PriorAngleError: " + priorError + "   ";
        loggingString += "CurrentAngleError: " + currentError + "   ";
        loggingString += "DrivePower: " + proportionPow + derivativePow + integralPow + "   ";
        loggingString += "derivativePower: " + derivativePow + "   ";
        loggingString += "proportionPower: " + proportionPow + "   ";
        loggingString += "CurrentAngle: " + imu.getAngularOrientation().firstAngle + "\n";
        return proportionPow + derivativePow + integralPow;
    }
    public double getAngleError(){
        return angleError;
    }
    public double getIntegralPower(){
        return integralPow;
    }
    public double getDerivativePower(){
        return derivativePow;
    }
    public double getProportionPower(){
        return proportionPow;
    }
    public void accumulateError(double delta_time, double angleError){
        accumulatedError += angleError * delta_time;
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
    //checks if the color sensor identifies tape color
    public void findTape() {
        while(currentBlueColor() < 70){ //blue tape TODO: get a num for "70"
            goToPosition(0, 0.8, 0, 0.8);
            if (avgPosition() > 700) {
                goToPosition(0.8, 0, 0.8, 0);
            }
        }



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
}
