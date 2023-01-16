package org.firstinspires.ftc.teamcode.State.Common;

import com.acmerobotics.dashboard.config.Config;
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
@Config
public class MecanumDrive {
    DcMotor fl, fr, bl, br;
    Telemetry telemetry;
    Thread driveThread;
    AtomicBoolean active;
    BNO055IMU imu;
    ColorRangeSensor colorTape;
    boolean timeout;

    private double accumulatedError;
    private double error;


    // creates/accesses file
    File loggingFile = AppUtil.getInstance().getSettingsFile("driveTelemetry.txt");
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

        colorTape.setGain(100);
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
        fr.setDirection(DcMotor.Direction.REVERSE);
        bl.setDirection(DcMotor.Direction.REVERSE);
        br.setDirection(DcMotor.Direction.FORWARD);
    }
    public void resetIMU(){
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // seehe calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu.initialize(parameters);
    }

    /*

    Current motor positions

    */

    public double getAngularOrientation() { return imu.getAngularOrientation().firstAngle;}
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
    /*

    SetPower methods

     */
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
    /*

     GoToPosition methods, including PID

     */

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

        setPower(flPow, frPow, blPow, brPow);
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

        int avgPosition = (int) (Math.abs(fl.getCurrentPosition()) + Math.abs(fr.getCurrentPosition()) + Math.abs(bl.getCurrentPosition()) + Math.abs(br.getCurrentPosition())) / 4;
        long time = System.currentTimeMillis();
        long difference;

        // If error is greater than zero, than run while loop
        while ((Math.abs(tics) - avgPosition) > 0) {
            setPower(flPow, frPow, blPow, brPow);
            avgPosition = (int) (Math.abs(fl.getCurrentPosition()) + Math.abs(fr.getCurrentPosition()) + Math.abs(bl.getCurrentPosition()) + Math.abs(br.getCurrentPosition())) / 4;
            difference = System.currentTimeMillis() - time;
            if(difference > 10000)break;
        }

        setPower(0, 0, 0, 0);
    }

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

        int position = (int) (Math.abs(fl.getCurrentPosition()) + Math.abs(fr.getCurrentPosition()) + Math.abs(bl.getCurrentPosition()) + Math.abs(br.getCurrentPosition())) / 4;

        long time = System.currentTimeMillis();
        long timeDifference = 0;

        double priorError = tics;
        double currentError = tics;

        //While error is greater than zero
        while ((Math.abs(tics) - position) > 0) {
            double drivePower;
            drivePower = PIDDrivePower(priorError, currentError, timeDifference);

            priorError = currentError;
            setPower(drivePower, drivePower, drivePower, drivePower);
            position = (int) (Math.abs(fl.getCurrentPosition()) + Math.abs(fr.getCurrentPosition()) + Math.abs(bl.getCurrentPosition()) + Math.abs(br.getCurrentPosition())) / 4;

            currentError = tics - position;
            accumulateError(timeDifference, currentError);

            timeDifference = System.currentTimeMillis() - time;
            if(timeDifference > tics * 1)break;
        }
        accumulatedError = 0;
        error = 0;
        setPower(0, 0, 0, 0);

    }

     /*
     Four Wheel PID below that doesn't work well due to hardware of turning while going straight
      */

    /*
    public static double flproportionCoefficient = 0.001;//0.75
    public static double flintegralCoefficient = 0;
    public static double flderivativeCoefficient = 1.2;//0.8
    public static double blproportionCoefficient = 0.001;//0.75
    public static double blintegralCoefficient = 0;
    public static double blderivativeCoefficient = 1.2;//0.8
    public static double frproportionCoefficient = 0.001;//0.75
    public static double frintegralCoefficient = 0;
    public static double frderivativeCoefficient = 1.2;//0.8
    public static double brproportionCoefficient = 0.001;//0.75
    public static double brintegralCoefficient = 0;
    public static double brderivativeCoefficient = 1.2;//0.8

    private double accumulatedErrorFl;
    private double accumulatedErrorBl;
    private double accumulatedErrorFr;
    private double accumulatedErrorBr;

    public void goToPositionPIDFlBlFrBr(int tics) {
        //front left, front right, back left, back right
        fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        br.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        fl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        int positionFl = (int)(Math.abs(fl.getCurrentPosition()));
        int positionBl = (int)(Math.abs(bl.getCurrentPosition()));
        int positionFr = (int)(Math.abs(fr.getCurrentPosition()));
        int positionBr = (int)(Math.abs(br.getCurrentPosition()));
        int avgPosition = (positionFl + positionBl + positionFr + positionBr)/4;

        double priorErrorFl = tics;
        double priorErrorBl = tics;
        double priorErrorFr = tics;
        double priorErrorBr = tics;

        double currentErrorFl = 0;
        double currentErrorBl = 0;
        double currentErrorFr = 0;
        double currentErrorBr = 0;

        long time = System.currentTimeMillis();
        long timeDifference = 0;

        //Encoder based gotoposition
        while ((Math.abs(tics) - positionFl) > 0 && (Math.abs(tics) - positionBl) > 0 && (Math.abs(tics) - positionFr) > 0 && (Math.abs(tics) - positionBr) > 0) {
            priorErrorFl = currentErrorFl;
            priorErrorBl = currentErrorBl;
            priorErrorFr = currentErrorFr;
            priorErrorBr = currentErrorBr;

            setPower(PIDfl(priorErrorFl, currentErrorFl, timeDifference), PIDfr(priorErrorFr, currentErrorFr, timeDifference), PIDbl(priorErrorBl, currentErrorBl, timeDifference), PIDbr(priorErrorBr, currentErrorBr, timeDifference));
            loggingString += " fltotalPower: " + PIDfl(priorErrorFl, currentErrorFl, timeDifference);
            loggingString += " bltotalPower: " + PIDbl(priorErrorBl, currentErrorBl, timeDifference);
            loggingString += " frtotalPower: " + PIDfr(priorErrorFr, currentErrorFr, timeDifference);
            loggingString += " brtotalPower: " + PIDbr(priorErrorBr, currentErrorBr, timeDifference);
            loggingString +="----------------------------------------------------------------------------\n";


            positionFl = (int)(Math.abs(fl.getCurrentPosition()));
            positionBl = (int)(Math.abs(bl.getCurrentPosition()));
            positionFr = (int)(Math.abs(fr.getCurrentPosition()));
            positionBr = (int)(Math.abs(br.getCurrentPosition()));
            avgPosition = (positionFl + positionBl + positionFr + positionBr)/4;

            currentErrorFl = tics - positionFl;
            currentErrorBl = tics - positionBl;
            currentErrorFr = tics - positionFr;
            currentErrorBr = tics - positionBr;

            accumulateErrorFlBlFrBr(timeDifference, currentErrorFl, currentErrorBl, currentErrorFr, currentErrorBr);

            timeDifference = System.currentTimeMillis() - time;

            if(timeDifference > tics * 1)break;
        }
        setPower(0, 0, 0, 0);

        // DDD

        accumulatedError = 0;
        error = 0;
        //loggingString +="----------------------------------------------------------------------------\n";

        loggingString += action.toUpperCase() + "\n";
        loggingString += "FL Position: " + getFLPosition() + "\n";
        loggingString += "FR Position: " + getFRPosition() + "\n";
        loggingString += "BL Position: " + getBLPosition() + "\n";
        loggingString += "BR Position: " + getBRPosition() + "\n";
        loggingString += "---------------------" + "\n";



        // loggingString += "Claw (Intake) Position: " + score.getClawPosition()


    }

    public double PIDfl(double priorError, double currentError, double timeChange){
        double proportionCoefficient = 0.001;//0.75
        double integralCoefficient = 0;
        double derivativeCoefficient = 1.2;//0.8
        double totalPower = currentError * flproportionCoefficient + ((currentError-priorError)/timeChange) * flderivativeCoefficient + getAccumulatedError() * flintegralCoefficient;
        error = currentError;
        return totalPower;
    }

    public double PIDbl(double priorError, double currentError, double timeChange){
        double proportionCoefficient = 0.001;//0.75
        double integralCoefficient = 0;
        double derivativeCoefficient = 1.2;//0.8
        double totalPower = currentError * blproportionCoefficient + ((currentError-priorError)/timeChange) * blderivativeCoefficient + getAccumulatedError() * blintegralCoefficient;
        error = currentError;
        return totalPower;
    }
    public double PIDfr(double priorError, double currentError, double timeChange){
        double proportionCoefficient = 0.001;//0.75
        double integralCoefficient = 0;
        double derivativeCoefficient = 1.2;//0.8
        double totalPower = currentError * frproportionCoefficient + ((currentError-priorError)/timeChange) * frderivativeCoefficient + getAccumulatedError() * frintegralCoefficient - (0.1);
        error = currentError;
        return totalPower;
    }
    public double PIDbr(double priorError, double currentError, double timeChange){
        double proportionCoefficient = 0.001;//0.75
        double integralCoefficient = 0;
        double derivativeCoefficient = 1.2;//0.8
        double totalPower = currentError * brproportionCoefficient + ((currentError-priorError)/timeChange) * brderivativeCoefficient + getAccumulatedError() * brintegralCoefficient;
        error = currentError;
        return totalPower;
    }

    public void accumulateErrorFlBlFrBr(double deltaTime, double errorFl, double errorBl, double errorFr, double errorBr){
        accumulatedErrorFl += errorFl * deltaTime;
        accumulatedErrorBl += errorBl * deltaTime;
        accumulatedErrorFr += errorFr * deltaTime;
        accumulatedErrorBr += errorBr * deltaTime;
    }

    public double getAccumulatedErrorFl(){
        return accumulatedErrorFl;
    }
    public double getAccumulatedErrorBl(){
        return accumulatedErrorBl;
    }
    public double getAccumulatedErrorFr(){
        return accumulatedErrorFr;
    }
    public double getAccumulatedErrorBr(){
        return accumulatedErrorBr;
    }

     */

    //TODO: Needs Testing
    public void goToPositionFBPID(int tics) {
        //front left, front right, back left, back right
        fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        br.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        fl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        int positionFl = (int)(Math.abs(fl.getCurrentPosition()));
        int positionBl = (int)(Math.abs(bl.getCurrentPosition()));
        int positionFr = (int)(Math.abs(fr.getCurrentPosition()));
        int positionBr = (int)(Math.abs(br.getCurrentPosition()));
        int avgPosition = (positionFl + positionBl + positionFr + positionBr)/4;


        double priorErrorFront = tics;
        double priorErrorBack = tics;

        double currentErrorFront = 0;
        double currentErrorBack = 0;

        long time = System.currentTimeMillis();
        long timeDifference = 0;

        //Encoder based gotoposition
        while ((Math.abs(tics) - positionFl) > 0 && (Math.abs(tics) - positionBl) > 0 && (Math.abs(tics) - positionFr) > 0 && (Math.abs(tics) - positionBr) > 0) {
            priorErrorFront = currentErrorFront;
            priorErrorBack = currentErrorBack;


            setPower(frontPow(priorErrorFront, currentErrorFront, timeDifference), frontPow(priorErrorFront, currentErrorFront, timeDifference), backPow(priorErrorBack, currentErrorBack, timeDifference), backPow(priorErrorBack, currentErrorBack, timeDifference));


            positionFl = (int)(Math.abs(fl.getCurrentPosition()));
            positionBl = (int)(Math.abs(bl.getCurrentPosition()));
            positionFr = (int)(Math.abs(fr.getCurrentPosition()));
            positionBr = (int)(Math.abs(br.getCurrentPosition()));
            avgPosition = (positionFl + positionBl + positionFr + positionBr)/4;

            currentErrorFront = tics - (positionFl + positionFr)/2;
            currentErrorBack = tics - (positionBl + positionBr)/2;


            timeDifference = System.currentTimeMillis() - time;
            if(timeDifference > tics * 1)break;
        }
        setPower(0, 0, 0, 0);
        accumulatedError = 0;
        error = 0;
    }


    /*

    Rotate PID's and Rotate methods Below

     */

    // IMU based PID turning, most optimal for 45 degree turning.
    public void turn45(double radians) {
        double initialAngle = imu.getAngularOrientation().firstAngle;

        double currentAngleError = angleWrap(radians);
        double priorAngleError = angleWrap(radians);

        ElapsedTime time = new ElapsedTime();
        double priorTime = time.milliseconds();
        double timeDifference = 0;

        // Relative turning
        while (Math.abs((imu.getAngularOrientation().firstAngle - initialAngle)) < Math.abs(radians)) {
            double drivePower = 0;
            if(currentAngleError == priorAngleError){
                //values stay the same
            }else{
                priorAngleError = angleWrap(radians - (imu.getAngularOrientation().firstAngle - initialAngle));
            }

            drivePower = PIDTurnPower45(priorAngleError, currentAngleError, timeDifference);
            setPower(-drivePower, drivePower, -drivePower, drivePower);


            accumulateError(timeDifference, currentAngleError);
            currentAngleError = angleWrap(radians - (imu.getAngularOrientation().firstAngle - initialAngle));


            timeDifference = time.milliseconds() - priorTime;
            if(time.milliseconds() - priorTime > 700) break;
        }
        //resetIMU();
        error = 0;
    }

    //IMU based PID turning, most optimal for 90 degree turns.
    public void turn90(double radians) {
        double initialAngle = imu.getAngularOrientation().firstAngle;

        double currentAngleError = angleWrap(radians);
        double priorAngleError = angleWrap(radians);

        ElapsedTime time = new ElapsedTime();
        double priorTime = time.milliseconds();
        double timeDifference = 0;

        // Relative turning
        while (Math.abs((imu.getAngularOrientation().firstAngle - initialAngle)) < Math.abs(radians)) {
            double drivePower = 0;
            if(currentAngleError == priorAngleError){
                //values stay the same
            }else{
                priorAngleError = angleWrap(radians - (imu.getAngularOrientation().firstAngle - initialAngle));
            }

            drivePower = PIDTurnPower90(priorAngleError, currentAngleError, timeDifference);
            setPower(-drivePower, drivePower, -drivePower, drivePower);


            accumulateError(timeDifference, currentAngleError);
            currentAngleError = angleWrap(radians - (imu.getAngularOrientation().firstAngle - initialAngle));


            timeDifference = time.milliseconds() - priorTime;
            if(time.milliseconds() - priorTime > 800) break;
        }
        //resetIMU();
        error = 0;
    }

    //IMU based PID turning, most optimal for 180 degree turning.
    public void turn180(double radians) {
        double initialAngle = imu.getAngularOrientation().firstAngle;

        double currentAngleError = angleWrap(radians);
        double priorAngleError = angleWrap(radians);

        ElapsedTime time = new ElapsedTime();
        double priorTime = time.milliseconds();
        double timeDifference = 0;

        // Relative turning
        while (Math.abs((imu.getAngularOrientation().firstAngle - initialAngle)) < Math.abs(radians)) {
            double drivePower = 0;
            drivePower = PIDTurnPower180(priorAngleError, currentAngleError, timeDifference);
            setPower(-drivePower, drivePower, -drivePower, drivePower);

            accumulateError(timeDifference, currentAngleError);
            priorAngleError = currentAngleError;
            currentAngleError = angleWrap(radians - (imu.getAngularOrientation().firstAngle - initialAngle));


            timeDifference = time.milliseconds() - priorTime;
            if(time.milliseconds() - priorTime > 700) break;
        }
        //resetIMU();
        error = 0;
    }

    public void turn(double radians) {
        double rad = radians;
        // Absolute turning
        while (Math.abs(imu.getAngularOrientation().firstAngle-radians) > 0.08) {
            double currentRadians = imu.getAngularOrientation().firstAngle;
            if (radians < currentRadians) {
                //turn right # of radians
                setPower(0.23, -0.23, 0.23, -0.23);
            } else if (currentRadians < radians) {
                //turn left # of radians
                setPower(-0.23, 0.23, -0.23, 0.23);
            } else {
               break;
            }
        }
    }
    /*
public void absTurnPID(double radians) {
    double startAngle = imu.getAngularOrientation().firstAngle;
    String shortestPath = "left";
    if(startAngle < 0){
        if((Math.abs(Math.PI - radians) + Math.abs(Math.PI - startAngle) > Math.abs(0 - radians) + Math.abs(0 - startAngle))){
            shortestPath = "left";
        }else{
            shortestPath = "right";
        }
    }else if(startAngle > 0){
        if(Math.abs(radians - startAngle) < Math.PI){
            shortestPath = "left";
        }else{
            shortestPath = "right";
        }
    }

    double priorError = angleWrap(radians - imu.getAngularOrientation().firstAngle);
    double currentError= angleWrap(radians - imu.getAngularOrientation().firstAngle);

    double priorTime = System.currentTimeMillis();
    double timeDifference = 0;
    // Absolute turning
    while (Math.abs(imu.getAngularOrientation().firstAngle-radians) > 0.08) {
        double currentRadians = imu.getAngularOrientation().firstAngle;

        double drivePower = PIDAbsTurnPower(priorError, currentError, timeDifference);

        if (radians < currentRadians-0.004) {
            if(startAngle > 0){
                setPower(-drivePower, drivePower, -drivePower, drivePower);
            }else{
                //turn right # of radians
                setPower(drivePower, -drivePower, drivePower, -drivePower);
            }

        } else if (currentRadians+0.004 < radians) {
            if(startAngle < 0){
                //setPower(drivePower, -drivePower, drivePower, -drivePower);
            }else {
                //turn left # of radians
                setPower(-drivePower, drivePower, -drivePower, drivePower);
            }
        } else {
            break;
        }


        priorError = currentError;
        currentError = angleWrap(radians - imu.getAngularOrientation().firstAngle);
        timeDifference = System.currentTimeMillis() - priorTime;
        if(timeDifference > 700)break;
    }
}

 */
    public void absTurnPID(double radians) {
        double startAngle = imu.getAngularOrientation().firstAngle;

        double priorError = angleWrap(radians - imu.getAngularOrientation().firstAngle);
        double currentError= angleWrap(radians - imu.getAngularOrientation().firstAngle);

        double priorTime = System.currentTimeMillis();
        double timeDifference = 0;
        // Absolute turning
        while (Math.abs(imu.getAngularOrientation().firstAngle-radians) > 0.08) {
            double currentRadians = imu.getAngularOrientation().firstAngle;

            double drivePower = PIDAbsTurnPower(priorError, currentError, timeDifference, startAngle, radians);
            if (radians < currentRadians-0.004) {
                //if(startAngle > 0){
                    //setPower(-drivePower, drivePower, -drivePower, drivePower);
                //}else{
                    //turn right # of radians
                    setPower(-drivePower, drivePower, -drivePower, drivePower);
                //}

            } else if (currentRadians+0.004 < radians) {
               // if(startAngle < 0){
                    //setPower(drivePower, -drivePower, drivePower, -drivePower);
                //}else {
                    //turn left # of radians
                    setPower(-drivePower, drivePower, -drivePower, drivePower);
                //}
            } else {
                break;
            }
            priorError = currentError;
            currentError = angleWrap(radians - imu.getAngularOrientation().firstAngle);
            timeDifference = System.currentTimeMillis() - priorTime;
            if(timeDifference > 700)break;
        }
    }
    /*

    Methods for updating PID(numbers for PIDs go here)

     */

    //Outputs the value of the power used in them method turn45()
    //You "plug" in the values for turn45() PID here
    public double PIDTurnPower45(double priorError, double currentError, double timeChange) {
        /*
        loggingString += "PriorAngleError: " + priorError + "   ";
        loggingString += "CurrentAngleError: " + currentError + "   ";
        loggingString += "DrivePower: " +  currentError * proportionCoefficient + ((currentError-priorError)/timeChange) * derivativeCoefficient + getAccumulatedError() * integralCoefficient + "   ";
        loggingString += "derivativePower: " + ((currentError-priorError)/timeChange) * derivativeCoefficient + "   ";
        loggingString += "proportionPower: " + getAccumulatedError() * integralCoefficient + "   ";
        loggingString += "CurrentAngle: " + imu.getAngularOrientation().firstAngle + "\n";
         */

        double proportionCoefficient = 0.845;//0.845
        double integralCoefficient = 0;
        double derivativeCoefficient = 0.5;//0.5

        error = currentError;
        return currentError * proportionCoefficient + ((currentError-priorError)/timeChange) * derivativeCoefficient + getAccumulatedError() * integralCoefficient;

    }

    //Outputs the value of the power used in them method turn90()
    //You "plug" in the values for turn90() PID here
    public double PIDTurnPower90(double priorError, double currentError, double timeChange) {
        /*
        loggingString += "PriorAngleError: " + priorError + "   ";
        loggingString += "CurrentAngleError: " + currentError + "   ";
        loggingString += "DrivePower: " +  currentError * proportionCoefficient + ((currentError-priorError)/timeChange) * derivativeCoefficient + getAccumulatedError() * integralCoefficient + "   ";
        loggingString += "derivativePower: " + ((currentError-priorError)/timeChange) * derivativeCoefficient + "   ";
        loggingString += "proportionPower: " + getAccumulatedError() * integralCoefficient + "   ";
        loggingString += "CurrentAngle: " + imu.getAngularOrientation().firstAngle + "\n";
         */

        double proportionCoefficient = 0.55;//0.6
        double integralCoefficient = 0;
        double derivativeCoefficient = 0.3;//0.3

        error = currentError;
        return currentError * proportionCoefficient + ((currentError-priorError)/timeChange) * derivativeCoefficient + getAccumulatedError() * integralCoefficient;
    }

    //Outputs the value of the power used in the method turn180()
    //You "plug" in the values for turn180() PID here
    public double PIDTurnPower180(double priorError, double currentError, double timeChange) {
         /*
        loggingString += "PriorAngleError: " + priorError + "   ";
        loggingString += "CurrentAngleError: " + currentError + "   ";
        loggingString += "DrivePower: " +  currentError * proportionCoefficient + ((currentError-priorError)/timeChange) * derivativeCoefficient + getAccumulatedError() * integralCoefficient + "   ";
        loggingString += "derivativePower: " + ((currentError-priorError)/timeChange) * derivativeCoefficient + "   ";
        loggingString += "proportionPower: " + getAccumulatedError() * integralCoefficient + "   ";
        loggingString += "CurrentAngle: " + imu.getAngularOrientation().firstAngle + "\n";
         */

        double proportionCoefficient = 0.47;//0.46
        double integralCoefficient = 0;
        double derivativeCoefficient = 0.18;//0.2

        error = currentError;
        return currentError * proportionCoefficient + ((currentError-priorError)/timeChange) * derivativeCoefficient + getAccumulatedError() * integralCoefficient;

    }
    public static double proportion = 0;
    public double PIDAbsTurnPower(double priorError, double currentError, double timeChange, double startAngle, double targetAngle){
        double proportionCoefficient = 0.55;//0.555
        double derivativeCoefficient = 0;//0.9
        if(Math.abs(startAngle - targetAngle) < Math.PI / 3){
            proportionCoefficient = 0.58;
        }else if(Math.abs(startAngle - targetAngle) < Math.PI / 2){
            proportionCoefficient = 0.56;
        }
        return currentError * proportionCoefficient + ((currentError-priorError)/timeChange) * derivativeCoefficient;

    }

    //TODO: Possibly needs reworking
    //Outputs the value of the power used in the method gotoPositionPID()
    //You "plug" in the values for gotoPositionPID() here
    public double PIDDrivePower(double priorError, double currentError, double timeChange){
          /*
        loggingString += "PriorError: " + priorError + "   ";
        loggingString += "CurrentError: " + currentError + "   ";
        loggingString += "derivativePower: " + ((currentError-priorError)/timeChange) * derivativeCoefficient + "   ";
        loggingString += "proportionPower: " + currentError * proportionCoefficient+ "   ";
        loggingString += "integralPower: " + getAccumulatedError() * integralCoefficient + "   ";
        loggingString += "DrivePower: " + totalPower + "\n";
         */

        double proportionCoefficient = 0.35;//0.35
        double integralCoefficient = 0;
        double derivativeCoefficient = 0.0011;//0.0011
        double totalPower = currentError * proportionCoefficient + ((currentError-priorError)/timeChange) * derivativeCoefficient + getAccumulatedError() * integralCoefficient;

        error = currentError;
        return totalPower;
    }

    //Outputs the value of the front power used in the method goToPositionFBPID()
    //You "plug" in the values for goToPositionFBPID() here
    public double frontPow(double priorError, double currentError, double timeChange){
        double proportionCoefficient = 0.001;
        double integralCoefficient = 0;
        double derivativeCoefficient = 0;
        return currentError * proportionCoefficient + ((currentError-priorError)/timeChange) * derivativeCoefficient + getAccumulatedError() * integralCoefficient;

    }

    //Outputs the value of the back power used in the method goToPositionFBPID()
    //You "plug" in the values for goToPositionFBPID() here
    public double backPow(double priorError, double currentError, double timeChange){
        double proportionCoefficient = 0.001;
        double integralCoefficient = 0;
        double derivativeCoefficient = 0;
        return currentError * proportionCoefficient + ((currentError-priorError)/timeChange) * derivativeCoefficient + getAccumulatedError() * integralCoefficient;

    }

    /*

  Color Sensor / Distance Sensor code for claw / Miscellaneous helper methods

   */

    public double getError(){
        return error;
    }
    public void accumulateError(double deltaTime, double error){
        accumulatedError += error * deltaTime;
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
    public double getPowerFl(){
        return fl.getPower();
    }
    public double getPowerFr(){
        return fr.getPower();
    }
    public double getPowerBl(){
        return bl.getPower();
    }
    public double getPowerBr(){
        return br.getPower();
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
    public double currentBlueColor() {

        return Math.round(colorTape.getNormalizedColors().blue * 100.0) / 100.0; // if current color is really high // 410
    }

    // for color sensor, returns the blue colo
    public double currentRedColor() {
        return Math.round(colorTape.getNormalizedColors().red * 100.0) / 100.0; // if current color is really high // 410
    }


    //checks if the color sensor identifies tape color
    public void findTape(String color, boolean strafe) {
        boolean temp = true;
        long time = System.currentTimeMillis();
        if (color.equalsIgnoreCase("blueleft")) {
            if (strafe) {
                goToPosition(0.2, 0.4, 0.4, 0.2, 300, "left");
            }
            while (currentBlueColor() < 0.52) { //blue tape
                if (temp) {
                    //If time takes too long, strafe right or left back to the tape
                    if((System.currentTimeMillis() - time)/1000 > 2.8){
                        goToPosition(0.3, -0.3, -0.3, 0.3);
                    }else{
                        goToPosition(0.3, 0, 0, 0.3);
                    }
                    // strafe diagonal left
                    //goToPosition(0.4, 0, 0, 0.4);
                    temp = false;
                }

            }
        } else if (color.equalsIgnoreCase("redright")) {
            if (strafe) {
                goToPosition(0.4, 0.2, 0.2, 0.4, 300, "right" );
            }
            while (currentRedColor() < 0.26) { //red tape
                if (temp) {
                    //If time takes too long, strafe right or left back to the tape
                    if((System.currentTimeMillis() - time)/1000 > 2.8){
                        goToPosition(-0.3, 0.3, 0.3, -0.3);
                    }else{
                        goToPosition(0, 0.3, 0.3, 0);
                    }
                    // strafe diagonal left
                    //goToPosition(0.4, 0, 0, 0.4);
                    temp = false;
                }

            }
        } else if (color.equalsIgnoreCase("blueright")) {
            if (strafe) {
                goToPosition(0.4, 0.2, 0.2, 0.4, 300, "right" );
            }
            while (currentBlueColor() < 0.52) { //blue tape
                if (temp) {
                    //If time takes too long, strafe right or left back to the tape
                    if((System.currentTimeMillis() - time)/1000 > 2.8){
                        goToPosition(-0.3, 0.3, 0.3, -0.3);
                    }else{
                        goToPosition(0, 0.3, 0.3, 0);
                    }
                    // strafe diagonal left
                    //goToPosition(0.4, 0, 0, 0.4);
                    temp = false;
                }

            }
        } else if (color.equalsIgnoreCase("redleft")) {
            if (strafe) {
                goToPosition(0.2, 0.4, 0.4, 0.2, 300, "left" );
            }
            while (currentRedColor() < 0.26) { //red tape
                if (temp) {
                    //If time takes too long, strafe right or left back to the tape
                    if((System.currentTimeMillis() - time)/1000 > 2.8){
                        goToPosition(-0.3, 0.3, 0.3, -0.3);
                    }else{
                        goToPosition(0.3, 0, 0, 0.3);
                    }
                    // strafe diagonal left
                    //goToPosition(0.4, 0, 0, 0.4);
                    temp = false;
                }

            }
        }
        goToPosition(0,0,0,0);
    }

    //checks if the color sensor identifies tape color
    public void findTapeMIDAUTO(String color) {
        boolean temp = true;
        long time = System.currentTimeMillis();
        if (color.equalsIgnoreCase("blueleft")) {
            while (currentBlueColor() < 0.52) { //blue tape
                if (temp) {
                    //If time takes too long, strafe right or left back to the tape
                    if((System.currentTimeMillis() - time)/1000 > 2.8){
                        goToPosition(0.3, -0.3, -0.3, 0.3);
                    }else{
                        goToPosition(0, 0.3, 0.3, 0);
                    }
                    // strafe diagonal left
                    //goToPosition(0.4, 0, 0, 0.4);
                    temp = false;
                }

            }
        } else if (color.equalsIgnoreCase("redright")) {
            while (currentRedColor() < 0.26) { //red tape
                if (temp) {
                    //If time takes too long, strafe right or left back to the tape

                    if((System.currentTimeMillis() - time)/1000 > 2.8){
                        goToPosition(-0.3, 0.3, 0.3, -0.3);
                    }else{
                        goToPosition(0.3, 0, 0, 0.3);
                    }
                    // strafe diagonal left
                    //goToPosition(0.4, 0, 0, 0.4);
                    temp = false;
                }

            }
        } else if (color.equalsIgnoreCase("blueright")) {
            while (currentBlueColor() < 0.52) { //blue tape
                if (temp) {
                    //If time takes too long, strafe right or left back to the tape
                    if((System.currentTimeMillis() - time)/1000 > 2.8){
                        goToPosition(-0.3, 0.3, 0.3, -0.3);
                    }else{
                        goToPosition(0.3, 0, 0, 0.3);
                    }
                    // strafe diagonal left
                    //goToPosition(0.4, 0, 0, 0.4);
                    temp = false;
                }

            }
        } else if (color.equalsIgnoreCase("redleft")) {

            while (currentRedColor() < 0.26) { //red tape
                if (temp) {
                    //If time takes too long, strafe right or left back to the tape
                    if((System.currentTimeMillis() - time)/1000 > 2.8){
                        goToPosition(0.3, -0.3, -0.3, 0.3);
                    }else{
                        goToPosition(0, 0.3, 0.3, 0);
                    }
                    // strafe diagonal left
                    //goToPosition(0.4, 0, 0, 0.4);
                    temp = false;
                }

            }
        }
        goToPosition(0,0,0,0);
    }


    // special method for after the 1+1 auto (finding tape is different)
    public void findTapeMulti(String color) {
        boolean temp = true;
        long time = System.currentTimeMillis();
        if (color.equalsIgnoreCase("blueleft")) {
            goToPosition(0.3, 0.6, 0.6, 0.3, 300, "left" );

            while (currentBlueColor() < 0.52) { //blue tape

                if (temp) {
                    //If time takes too long, strafe right or left back to the tape
                    if((System.currentTimeMillis() - time)/1000 > 2.8){
                        goToPosition(-0.3, 0.3, 0.3, -0.3);
                    }else{
                        goToPosition(0.3, -0.3, -0.3, 0.3);
                    }

                    // strafe diagonal left
                    //goToPosition(0.4, 0, 0, 0.4);
                    temp = false;
                }

            }
        } else if (color.equalsIgnoreCase("redright")) {
            goToPosition(0.6, 0.3, 0.3, 0.6, 300, "left" );
            while (currentRedColor() < 0.26) { //red tape


                if (temp) {
                    //If time takes too long, strafe right or left back to the tape
                    if((System.currentTimeMillis() - time)/1000 > 2.8){
                        goToPosition(0.3, -0.3, -0.3, 0.3);
                    }else{
                        goToPosition(-0.3, 0.3, 0.3, -0.3);
                    }
                    // strafe diagonal left
                    //goToPosition(0.4, 0, 0, 0.4);
                    temp = false;
                }

            }
        } else if (color.equalsIgnoreCase("redleft")) {
            goToPosition(0.3, 0.6, 0.6, 0.3, 300, "left" );


            while (currentRedColor() < 0.26) { //blue tape
                if (temp) {
                    //If time takes too long, strafe right or left back to the tape
                    if((System.currentTimeMillis() - time)/1000 > 2.8){
                        goToPosition(-0.3, 0.3, 0.3, -0.3);
                    }else{
                        goToPosition(0.3, -0.3, -0.3, 0.3);
                    }

                    // strafe diagonal left
                    //goToPosition(0.4, 0, 0, 0.4);
                    temp = false;
                }

            }
        } else if (color.equalsIgnoreCase("blueright")) {
            goToPosition(0.6, 0.3, 0.3, 0.6, 300, "left" );

            while (currentBlueColor() < 0.52) { //red tape

                if (temp) {
                    //If time takes too long, strafe right or left back to the tape
                    if((System.currentTimeMillis() - time)/1000 > 2.8){
                        goToPosition(0.3, -0.3, -0.3, 0.3);
                    }else{
                        goToPosition(-0.3, 0.3, 0.3, -0.3);
                    }
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
    public double currentAngle(){
        return imu.getAngularOrientation().firstAngle;
    }

    // returns the average tics for mecanum wheels
    public int avgPosition(){
        return (int)(Math.abs(fl.getCurrentPosition()) + Math.abs(fr.getCurrentPosition()) + Math.abs(bl.getCurrentPosition()) + Math.abs(br.getCurrentPosition()))/4;
    }
    // returns the average tics for mecanum wheels (inputed)
    public int avgPosition(int fl, int fr, int bl, int br){
        return (int)(Math.abs(fl) + Math.abs(fr) + Math.abs(bl) + Math.abs(br))/4;
    }

    /*
    Got to position useing thread, for more accuracy, does not work , do not use
     */
    /*
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
}
