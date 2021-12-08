package org.firstinspires.ftc.teamcode.TankDriveCode.Auto;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDCoefficients;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.MecanumCode.Common.OpModeWrapper;
import org.firstinspires.ftc.teamcode.MecanumCode.Common.Vector2D;

import java.io.File;
import java.io.FileNotFoundException;
import java.io.PrintStream;
import java.util.Arrays;

public class PIDTestDriveTrain {
    private static final double ANGULAR_TOLERANCE = 0.05;
    final double COUNTS_PER_INCH = 920.111004;

    /*
    This has most of the relevant information regarding a 4 wheel Mecanum DriveTrain,
    which is the most used DriveTrain in FTC
     */

    DcMotor fl;
    DcMotor fr;
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

    public PIDTestDriveTrain(HardwareMap hardwareMap) throws FileNotFoundException {
        fl  = hardwareMap.get(DcMotor.class, "LeftFrontDrive");
        fr = hardwareMap.get(DcMotor.class, "RightFrontDrive");

        correctMotors();

    }



    private void correctMotors() {
        fl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //bl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //br.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }









    public String getMotorPower(){
        return "fl: " + fl.getPower() + "\n" +
                "fr: " + fr.getPower() + "\n";
                //"bl: " + bl.getPower() + "\n" +
                //"br: " + br.getPower();
    }



    private void brake() {
        fl.setPower(0);
        fr.setPower(0);
        //bl.setPower(0);
        //br.setPower(0);
    }


    public DcMotor[] getMotors(){
        return new DcMotor[]{fl,fr/*,bl,br*/};
    }


    /*
    gets the robot driving in a specified direction
     */
    public void setPower(Vector2D velocity, double turnValue){
        turnValue = -turnValue;
        double direction = velocity.getDirection();


        double power = velocity.magnitude();

        double angle = direction + 3*Math.PI / 4.0;
        double sin = Math.sin(angle);
        double cos = Math.cos(angle);

        setPower((power * sin - turnValue),(power * cos + turnValue)/*,
                (power * cos - turnValue), (power * sin + turnValue)*/);
    }


    /**
     * a method for driving in the cardinal directions of the 3 dimensional space (x,y,theta)
     * @param desiredVelocity velocity in tics / sec the robot should attempt to reach
     * @param tics the distance the robot should travel in tics
     * @param movement the type of movement the robot should perform, straight, strafe, or rotate
     */

    //TODO write a tics to inches conversion as well as a tics to degrees conversion
    public void driveAuto(double desiredVelocity, int tics, MovementType movement){
        fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //bl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //br.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //bl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //br.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        OpModeWrapper currentOpMode = OpModeWrapper.currentOpMode();

        double startTime = currentOpMode.time;
        double lastError = 0;
        double velocity = 0.25;
        previousPositions = new double[]{0,0,0,0};
        double[] currentPositions = new double[]{0, 0, 0, 0};

        double average = computeAvg(currentPositions);
        double error = 0;
        while(Math.abs(average) <= Math.abs(tics / 2)){
            currentPositions = new double[]{fl.getCurrentPosition(), fr.getCurrentPosition()/*, bl.getCurrentPosition(), br.getCurrentPosition()*/};
            System.out.println("hi");

            currentOpMode.telemetry.addData("currentPositions: ", Arrays.toString(currentPositions));
            currentOpMode.telemetry.addData("error: ", error);

            average = computeAvg(currentPositions);
            error = average - ((6 * tics) / 13.0);
            double P = PID.p * error;

            while(P * 10 < 1){
                P *= 10;
            }
            if(error < 0 && (1 + P) <= 0.25){
                 velocity = 0.25;
            }else if(error < 0 && (1 + P) > 0.25 && (1 + P) < 1){
                 velocity = 1 + P;
            }else{
                velocity = 1;
            }


            if(tics < 0){
                setPowerAuto(-velocity, movement);
            }else{
                setPowerAuto(velocity, movement);
            }



        }


        while(Math.abs(average) < Math.abs(tics - 10) && Math.abs(average) > Math.abs(tics / 2)){
            currentPositions = new double[]{fl.getCurrentPosition(), fr.getCurrentPosition()/*, bl.getCurrentPosition(), br.getCurrentPosition()*/};

            average = computeAvg(currentPositions);
            error = ((12 * tics) / 13.0) - average;
            currentOpMode.telemetry.addData("currentPositions: ", Arrays.toString(currentPositions));
            currentOpMode.telemetry.addData("error: ", error);

            double P = PID.p * error;

            while(P * 10 < 1){
                P *= 10;
            }


            if(error < 0.25 ){
                velocity = 0.25;
             }else{
                 velocity = P;
             }

            setPowerAuto(velocity, movement);




        }




        /*

        while(isFar(tics) && currentOpMode.opModeIsActive()){
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







            if(tics > 0) {
                previousVelocity = setPowerAuto(desiredVelocity, movement);
            } else {
                previousVelocity = setPowerAuto(-desiredVelocity, movement);
            }

            //previousPositions = new double[]{currentPositions[0], currentPositions[1], currentPositions[2], currentPositions[3]};
            //previousError = velocityError;





        }

         */





    }


    private double TIC_TOLERANCE = 25;
    private boolean isFar(int tics){
        return Math.abs(tics - fl.getCurrentPosition()) > TIC_TOLERANCE && Math.abs(tics - fr.getCurrentPosition()) > TIC_TOLERANCE
                /*&& Math.abs(tics - bl.getCurrentPosition()) > TIC_TOLERANCE && Math.abs(tics - br.getCurrentPosition()) > TIC_TOLERANCE*/;
    }



    public enum MovementType{
        STRAIGHT, STRAFE, ROTATE
    }

    private double computeDeltas(double[] currentPositions){
        double flDelta = Math.abs(currentPositions[0]) - Math.abs(previousPositions[0]);
        double frDelta = Math.abs(currentPositions[1]) - Math.abs(previousPositions[1]);
        double blDelta = Math.abs(currentPositions[2]) - Math.abs(previousPositions[2]);
        double brDelta = Math.abs(currentPositions[3]) - Math.abs(previousPositions[3]);
        return (flDelta + frDelta + blDelta + brDelta) / 4.0;
    }

    private double computeAvg(double[] currentPositions){

        return (Math.abs(currentPositions[0]) + Math.abs(currentPositions[1]) /*currentPositions[2] + currentPositions[3]*/) / 2.0;
    }





    public void setPower(double flPow, double frPow/*, double blPow, double brPow*/) {
        fl.setPower(-flPow);
        fr.setPower(-frPow) ;
        //bl.setPower(blPow);
        //br.setPower(brPow);
    }

    public double setPowerAuto(double power, MovementType movement) {
        if(movement == MovementType.STRAIGHT) {
            setPower(power, power/*, power, power*/);
        }else if(movement == MovementType.STRAFE){
            setPower(power, -power/*, -power, power*/);
        }else if(movement == MovementType.ROTATE){
            setPower(power, -power/*, power, -power*/);
        }
        return power;
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







}