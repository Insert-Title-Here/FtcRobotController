package org.firstinspires.ftc.teamcode.MecanumCode.Common;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDCoefficients;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;
import java.io.FileNotFoundException;
import java.io.PrintStream;
import java.sql.Wrapper;
import java.util.Arrays;

/**
 * Configuration:
 *
 */
public class MecanumDriveTrain {
    private static final double ANGULAR_TOLERANCE = 0.05;
    final double COUNTS_PER_INCH = 920.111004;

    /*
    This has most of the relevant information regarding a 4 wheel Mecanum DriveTrain,
    which is the most used DriveTrain in FTC
     */

    public DcMotor fl, fr, bl, br;
    BNO055IMU imu;

    File loggingFile = AppUtil.getInstance().getSettingsFile("telemetry.txt");
    String loggingString;

    private double previousVelocity;
    private double[] previousPositions;
    private double previousError;

    /**
     * PID Constants
     *
     */
    final double pVelocity = 0.000725; //0.000725
    final double dVelocity  = 0.047; //0.027
    PIDCoefficients PID = new PIDCoefficients(0.002, 0, 0);

    public MecanumDriveTrain(HardwareMap hardwareMap) throws FileNotFoundException {
        fl = hardwareMap.dcMotor.get("FrontLeftDrive");
        fr = hardwareMap.dcMotor.get("FrontRightDrive");
        bl = hardwareMap.dcMotor.get("BackLeftDrive");
        br = hardwareMap.dcMotor.get("BackRightDrive");
        correctMotors();

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.RADIANS;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");

        imu.initialize(parameters);


    }



    private void correctMotors() {
        fl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }









    public String getMotorPower(){
        return "fl: " + fl.getPower() + "\n" +
                "fr: " + fr.getPower() + "\n" +
                "bl: " + bl.getPower() + "\n" +
                "br: " + br.getPower();
    }



    private void brake() {
        fl.setPower(0);
        fr.setPower(0);
        bl.setPower(0);
        br.setPower(0);
    }


    public DcMotor[] getMotors(){
        return new DcMotor[]{fl,fr,bl,br};
    }


    /*
    gets the robot driving in a specified direction
     */
    public void setPower(Vector2D velocity, double turnValue, boolean isSwapped){
        turnValue = -turnValue;
        double direction = velocity.getDirection();


        double power = velocity.magnitude();

        double angle = direction + 3*Math.PI / 4.0;
        double sin = Math.sin(angle);
        double cos = Math.cos(angle);

        if(!isSwapped) {
            setPower((power * sin - turnValue), (power * cos + turnValue),
                    (power * cos - turnValue), (power * sin + turnValue));
        } else {
            setPower(-(power * sin - turnValue), -(power * cos + turnValue),
                    -(power * cos - turnValue), -(power * sin + turnValue));
        }
    }


    /**
     * a method for driving in the cardinal directions of the 3 dimensional space (x,y,theta)
     * @param desiredVelocity velocity in tics / sec the robot should attempt to reach
     * @param tics the distance the robot should travel in tics
     * @param movement the type of movement the robot should perform, straight, strafe, or rotate
     */

    //TODO write a tics to inches conversion as well as a tics to degrees conversion
    public synchronized void driveAuto(double desiredVelocity, int tics, MovementType movement){

        fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        br.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        OpModeWrapper currentOpMode = OpModeWrapper.currentOpMode();
        double startTime = currentOpMode.time;
        /*
        double startTime = currentOpMode.time;
        double lastError = 0;
        previousPositions = new double[]{0,0,0,0};
        double[] currentPositions = new double[]{0, 0, 0, 0};

        double average = computeAvg(currentPositions);
        double error = 0;
        while(average <= tics / 2){
            currentPositions = new double[]{fl.getCurrentPosition(), fr.getCurrentPosition(), bl.getCurrentPosition(), br.getCurrentPosition()};

            currentOpMode.telemetry.addData("currentPositions: ",currentPositions);
            currentOpMode.telemetry.addData("error: ",error);

            average = computeAvg(currentPositions);
            error = average - (6 * tics) / 13;
            double P = PID.p * error;
            if(error < 0 && 1 + P <= 0.25){
                setPowerAuto(0.25, movement);
            }else if(error < 0 && 1 + P > 0.25){
                setPowerAuto(1 + P, movement);
            }else{
                setPowerAuto(1, movement);
            }

            lastError = error;

        }

        while(average < tics - 10){
            currentPositions = new double[]{fl.getCurrentPosition(), fr.getCurrentPosition(), bl.getCurrentPosition(), br.getCurrentPosition()};

            error = computeAvg(currentPositions) - (12 * tics) / 13;
            currentOpMode.telemetry.addData("currentPositions: ",currentPositions);
            currentOpMode.telemetry.addData("error: ",error);


            if(error > 0){
                 setPowerAuto(0.1, movement);
             }else{
                 double P = Math.abs(PID.p * error);
                 setPowerAuto(P, movement);
             }


             lastError = error;

        }

         */

        if(tics < 0) {
            desiredVelocity *= -1;
        }
        double elapsedTime = 0;

        while(isFar(tics) && currentOpMode.opModeIsActive() && elapsedTime < 5.0){
            elapsedTime = currentOpMode.time - startTime;
            setPowerAuto(desiredVelocity, movement);
        }

        brake();




        previousVelocity = 0;
        previousPositions = new double[]{0,0,0,0};
        previousError = 0;
        //double previousTime = startTime;

        while(isFar(tics) && currentOpMode.opModeIsActive()){

            double currentTime = currentOpMode.time;

            //double dt = currentTime - previousTime;


            double currentVelocity;
            //double deltaPositions = computeDeltas(currentPositions);

            /*
            if(currentTime - startTime < 0.01){ //TODO not sure this will work the way we want it to, the top may invoke every time causing bad things to happen.
                currentVelocity = 0;
            }else {
                currentVelocity = deltaPositions / dt;
            }


            //previousTime = currentTime;
            currentOpMode.telemetry.addData("dt", dt);
            loggingString += "dt: " + dt + "\n";
            currentOpMode.telemetry.addData("velocity",currentVelocity);
            loggingString += "velocity: " + currentVelocity + "\n";
            currentOpMode.telemetry.addData("deltaPositions: ",deltaPositions);
            loggingString += "deltaPositions: " + deltaPositions + "\n";
            currentOpMode.telemetry.addData("Position :", Arrays.toString(currentPositions));
            loggingString += "Position: " + Arrays.toString(currentPositions) + "\n";
            currentOpMode.telemetry.update();



            double velocityError = desiredVelocity - currentVelocity;
            double dError = velocityError - previousError;
            double output = velocityError * pVelocity + dError * dVelocity;
            double passedValue = previousVelocity + output;
            if(passedValue > 1.0){
                desiredVelocity = currentVelocity;
                passedValue = 1.0;
            }else if(passedValue < -1.0){
                desiredVelocity = currentVelocity;
                passedValue = -1.0;
            }

             */


            if(tics > 0) {
                previousVelocity = setPowerAuto(desiredVelocity, movement);
            } else {
                previousVelocity = setPowerAuto(-desiredVelocity, movement);
            }

            //previousPositions = new double[]{currentPositions[0], currentPositions[1], currentPositions[2], currentPositions[3]};
            //previousError = velocityError;



        }



    }


    private double TIC_TOLERANCE = 25;
    private boolean isFar(int tics){
        return Math.abs(tics - fl.getCurrentPosition()) > TIC_TOLERANCE && Math.abs(tics - fr.getCurrentPosition()) > TIC_TOLERANCE
                && Math.abs(tics - bl.getCurrentPosition()) > TIC_TOLERANCE && Math.abs(tics - br.getCurrentPosition()) > TIC_TOLERANCE;
    }



    public enum MovementType{
        STRAIGHT, STRAFE, ROTATE, LDIAGONAL, RDIAGONAL
    }

    private double computeDeltas(double[] currentPositions){
        double flDelta = Math.abs(currentPositions[0]) - Math.abs(previousPositions[0]);
        double frDelta = Math.abs(currentPositions[1]) - Math.abs(previousPositions[1]);
        double blDelta = Math.abs(currentPositions[2]) - Math.abs(previousPositions[2]);
        double brDelta = Math.abs(currentPositions[3]) - Math.abs(previousPositions[3]);
        return (flDelta + frDelta + blDelta + brDelta) / 4.0;
    }

    private double computeAvg(double[] currentPositions){

        return (currentPositions[0] + currentPositions[1] + currentPositions[2] + currentPositions[3]) / 4.0;
    }





    public void setPower(double flPow, double frPow, double blPow, double brPow) {
        fl.setPower(-flPow);
        fr.setPower(-frPow) ;
        bl.setPower(blPow);
        br.setPower(brPow);
    }

    public double setPowerAuto(double power, MovementType movement) {
        if(movement == MovementType.STRAIGHT) {
            setPower(power, power, power, power);
        }else if(movement == MovementType.STRAFE){
            setPower(power, -power, -power, power);
        }else if(movement == MovementType.ROTATE){
            setPower(power, -power, power, -power);
        }else if(movement == MovementType.LDIAGONAL){
            setPower(power, 0, 0, power);
        }else if(movement == MovementType.RDIAGONAL){
            setPower(0, power, power, 0);
        }
        return power;
    }

    public double getAngle(){
        return imu.getAngularOrientation().firstAngle;
    }



    private int getSign(double num){
        if(num < 0){
            return -1;
        }else {
            return 1;
        }
    }


    public void writeLoggerToFile(){
        try{
            PrintStream toFile = new PrintStream(loggingFile);
            toFile.println(loggingString);
        }catch(FileNotFoundException e){
            e.printStackTrace();
        }
    }


    public void tankRotate(double radians, double power){

        //double changeRads = Math.abs(radians) - Math.PI;



        if(radians > imu.getAngularOrientation().firstAngle){
            power *= -1;
        }

        //double position = Math.abs(radians + imu.getAngularOrientation().firstAngle);


        while(Math.abs(imu.getAngularOrientation().firstAngle - radians) >  (Math.PI/32)){
            setPowerAuto(power, MovementType.ROTATE);

        }

        brake();

    }

    public void tinyRotate(rotatePower power) {
        if (power == rotatePower.Negative) {
            fl.setPower(-0.5);
            fr.setPower(0.5);
            bl.setPower(0);
        }else{
            //br.setPower(0.3);
            //fr.setPower(0);
            fl.setPower(0.5);
            fr.setPower(-0.5);
            bl.setPower(0);
        }


    }

    public void bothRotate(rotatePower power) {
        if (power == rotatePower.Negative) {
            bl.setPower(-0.3);
            br.setPower(0.3);
            fr.setPower(0.3);
        }else{
            bl.setPower(0.3);
            br.setPower(0.3);
            fr.setPower(-0.3);
        }


    }

    public enum rotatePower{
        Positive, Negative
    }


    public void tankSetPower(double linear, double rotate){
        setPower(linear + rotate, linear - rotate, linear + rotate, linear - rotate);
    }






}