package org.firstinspires.ftc.teamcode.State.Common;

import static java.lang.Thread.sleep;

import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;
import java.io.FileNotFoundException;
import java.io.PrintStream;

public class ScoringSystem {
    private DcMotor liftMotorLeft;
    private DcMotor liftMotorRight;
    private Servo claw;
    private Servo camTurn;
    //private Servo uprighter;
    MecanumDrive drive;
    ColorRangeSensor colorCone;
    Telemetry telemetry;
    Constants constant;
    File loggingFile = AppUtil.getInstance().getSettingsFile("telemetry.txt");

    public String loggingString;
    //public static double proportion = 0.0;
    //public static double derivative = 0.0;
    public ScoringSystem(HardwareMap hardwareMap, Telemetry telemetry) {
        /* the below is init*/
        constant = new Constants();
        claw = hardwareMap.get(Servo.class, "claw");
        camTurn = hardwareMap.get(Servo.class, "camTurn");
        //uprighter = hardwareMap.get(Servo.class, "uprighter");
        liftMotorLeft = hardwareMap.get(DcMotor.class, "liftMotorLeft");
        liftMotorRight = hardwareMap.get(DcMotor.class, "liftMotorRight");
        colorCone = hardwareMap.get(ColorRangeSensor.class, "colorCone");
        drive = new MecanumDrive(hardwareMap, telemetry);
        this.telemetry = telemetry;
        // reset encoder's tics for liftMotor (leave commented unless you need to reset the encoder for
        liftMotorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        //Not actually without encoder (just doesn't use given PID)
        liftMotorRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftMotorLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        // when the power is zero, it'll resist movement/change
        liftMotorRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftMotorLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        liftMotorRight.setDirection(DcMotor.Direction.REVERSE);
        liftMotorLeft.setDirection(DcMotor.Direction.FORWARD);


    }

    //goes to given tics
    public void goToPosition(int tics, double power) {
        int motorPosition = liftMotorLeft.getCurrentPosition();
        if (motorPosition > tics) {
            power *= -1;
        }
        long time = System.currentTimeMillis();
        long timeChange= 0;
        double proportionPow;
        double derivativePow;
        while (Math.abs(Math.abs(tics)-motorPosition) > 10) {

            //set power to zero if tics pretty high and power continually being used, stops lift
            //system from breaking itself from trying to go past mechanical max
            if((Math.abs(timeChange) > 2000)){
                liftMotorLeft.setPower(0);
                liftMotorRight.setPower(0);
                //stops while loop
                break;
            }else{
                if(Math.abs(tics) - motorPosition < 0){
                    liftMotorLeft.setPower(power);
                    liftMotorRight.setPower(power);

                }else{
                    liftMotorLeft.setPower(power*0.8);
                    liftMotorRight.setPower(power*0.8);
                }
                motorPosition = liftMotorLeft.getCurrentPosition();
            }
            timeChange =  System.currentTimeMillis() - time;
        }


        liftMotorLeft.setPower(0);
        liftMotorRight.setPower(0);


    }


    public void goToPositionPID(int tics, double power) {
        int motorPosition = liftMotorLeft.getCurrentPosition();
        if (motorPosition > tics) {
            power *= -1;
        }
        long time = System.currentTimeMillis();
        long timeChange= 0;

        double priorError = tics;
        double currentError = tics;

        while (Math.abs(Math.abs(tics)-motorPosition) > 10) {

            //set power to zero if tics pretty high and power continually being used, stops lift
            //system from breaking itself from trying to go past mechanical max
            if((Math.abs(timeChange) > 1500)){
                liftMotorLeft.setPower(0);
                liftMotorRight.setPower(0);
                //stops while loop
                break;
            }else{
                liftMotorLeft.setPower(PIDLiftPower(priorError, currentError, timeChange, tics));
                liftMotorRight.setPower(PIDLiftPower(priorError, currentError, timeChange, tics));
                motorPosition = liftMotorLeft.getCurrentPosition();
                priorError = currentError;
                currentError = tics - motorPosition;
            }
            timeChange =  System.currentTimeMillis() - time;
        }


        liftMotorLeft.setPower(0);
        liftMotorRight.setPower(0);


    }
    public double PIDLiftPower(double priorError, double currentError, double timeChange, int tics) {
        double proportionCoefficient;
        if(liftMotorLeft.getCurrentPosition() > tics){
            proportionCoefficient = 0.006;
        }else{
            proportionCoefficient = 0.0119;
        }
        //0.0119
        //double derivativeCoefficient = 0.0;//0.2
       // proportionCoefficient = proportion;
        //derivativeCoefficient = derivative;
        /*
        loggingString += "PriorAngleError: " + priorError + "   ";
        loggingString += "CurrentAngleError: " + currentError + "   ";
        loggingString += "DrivePower: " +  currentError * proportionCoefficient + ((currentError-priorError)/timeChange) * derivativeCoefficient + "   ";
        loggingString += "derivativePower: " + ((currentError-priorError)/timeChange) * derivativeCoefficient + "   ";
        loggingString += "proportionPower: " + currentError * proportionCoefficient + "   ";
         */

        return currentError * proportionCoefficient /*+ ((currentError-priorError)/timeChange) * derivative*/;

    }



    public void writeLoggerToFile(){
        try{
            PrintStream toFile = new PrintStream(loggingFile);
            toFile.println(loggingString);
        }catch(FileNotFoundException e){
            e.printStackTrace();
        }
    }
    public void writeLoggerToFile(File file, String log){
        try{
            PrintStream toFile = new PrintStream(file);
            toFile.println(log);
        }catch(FileNotFoundException e){
            e.printStackTrace();
        }
    }
    public void setPower(double power){
        liftMotorLeft.setPower(power);
        liftMotorRight.setPower(power);
    }
    public double getRightPower(){
        return liftMotorLeft.getPower();
    }
    public double getLeftPower(){
        return liftMotorLeft.getPower();
    }


    public void setClawPosition(double position) {
        claw.setPosition(position);
    }

    public void setCamPosition(double position) {
        camTurn.setPosition(position);
    }

    //public void setUprighterPosition(double position) { uprighter.setPosition(position);}

    public int getEncoderPosition() {
        return (liftMotorLeft.getCurrentPosition() + liftMotorRight.getCurrentPosition())/2;
    }



    public double getClawPosition() {
        return claw.getPosition();
    }
    public double getCamPosition() {
        return camTurn.getPosition();
    }
    //public double getUprighterPosition() { return uprighter.getPosition(); }

    public void resetLiftEncoder(){
         liftMotorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public boolean isBusy(){
        return liftMotorLeft.isBusy() && liftMotorRight.isBusy();
    }
    //returns numerical value of how "blue it is" from the color sensor
    public int currentBlueColor() {
        return colorCone.blue(); // if current color is really high // 410
    }
    //returns the numerical value of how "red it is" from the color sensor
    public int currentRedColor() {
        return colorCone.red(); // if current color is really high // 177
    }
    // Uses color sensor to grab cone
    public boolean grabCone() throws InterruptedException {
        if (colorCone.getDistance(DistanceUnit.CM) < 2.5) {
            // grab cone
            setClawPosition(constant.getClawClosePos());
            sleep(500);
            // lift up
            goToPosition(getEncoderPosition() + 200, 0.9);
            telemetry.addData("distance", colorCone.getDistance(DistanceUnit.CM));
            telemetry.update();
            return true;

        }
        return false;
    }
    //uses color sensor to grab cone(this one is used when trying to grab from the stack of 5 cones
    // tele version
    public boolean grabCone(boolean stack) throws InterruptedException {
        if (colorCone.getDistance(DistanceUnit.CM) < 2.5) {
            // grab cone
            setClawPosition(constant.getClawClosePos());
            sleep(500);
            // lift up
            if(stack){
                goToPosition(getEncoderPosition() + 330, 1);
            }else{
                goToPosition(constant.getHeightLow(), 1);
            }
            telemetry.addData("distance", colorCone.getDistance(DistanceUnit.CM));
            telemetry.update();
            return true;

        }
        return false;
    }


    // auto version
    public void grabConeAuto() {
        boolean temp = true;
        drive.goToPosition(0.2, 0.2, 0.2, 0.2);
        while (temp) {
            if (colorCone.getDistance(DistanceUnit.CM) < 3.5) {
                drive.goToPosition(0, 0, 0, 0);
                // grab cone
                setClawPosition(constant.getClawClosePos());
                try {
                    sleep(300);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
                drive.goToPosition(-0.4, -0.4, -0.4, -0.4, 30, "move backwards");
                // lift up
                goToPositionPID(constant.getHeightLow(), 0.7);

                telemetry.addData("distance", colorCone.getDistance(DistanceUnit.CM));
                telemetry.update();
                temp = false;
            }
        }
    }
    boolean high = false;
    boolean mid = false;
    boolean low = false;
    public void setScoreBoolean(boolean h, boolean m, boolean l){
        high = h;
        mid = m;
        low = l;
    }
    public boolean scoreHigh(){
        return high;
    }
    public boolean scoreMid(){
        return mid;
    }
    public boolean scoreLow(){
        return low;
    }
    public double getDistance() {
        return colorCone.getDistance(DistanceUnit.CM);
    }

    int height1 = 170;
    int height2 = 130;
    int height3 = 80;
    int height4 = 50;
    int height5 = 5;
    int currentHeight = 0;
    public void stackUp(){
        /*
        if(currentHeight == 0){
            goToPosition(height1, 0.1);
            currentHeight = height1;
        }else if(currentHeight == height5){
            goToPosition(height4, 0.1);
            currentHeight = height4;
        }else if(currentHeight == height4){
            goToPosition(height3, 0.1);
            currentHeight = height3;
        }else if(currentHeight == height3){
            goToPosition(height2, 0.1);
            currentHeight = height2;
        }else if(currentHeight == height2){
            goToPosition(height1, 0.1);
            currentHeight = height1;
        }

         */
        if(currentHeight == 0){
            goToPosition(160, 0.6);
            currentHeight = 160;
        }else{
            if(getEncoderPosition() < 210){
                goToPosition(currentHeight-35, 0.6);
                currentHeight -= 35;
            }
        }
    }

    public void stackDown(){
        /*
        if(currentHeight == height1){
            goToPosition(height2, 0.1);
            currentHeight = height2;
        }else if(currentHeight == height2){
            goToPosition(height3, 0.1);
            currentHeight = height3;
        }else if(currentHeight == height3){
            goToPosition(height4, 0.1);
            currentHeight = height4;
        }else if(currentHeight == height4){
            goToPosition(height5, 0.1);
            currentHeight = height5;
        }

         */
        goToPosition(currentHeight + 35, 0.6);
    }

}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/* Currently Does Not Work
    //goes to given tics
    public void goToPosition(int tics, double power) {
        int motorPositionLeft = liftMotorLeft.getCurrentPosition();
        int motorPositionRight = liftMotorRight.getCurrentPosition();

        if (motorPositionLeft > tics) {
            power *= -1;
        }
        if (motorPositionRight > tics) {
            power *= -1;
        }
        long time = System.currentTimeMillis();
        long timeChange= 0;
        double proportionPow;
        double derivativePow;
        while (Math.abs(Math.abs(tics)-motorPositionLeft) > 10 && Math.abs(Math.abs(tics)-motorPositionRight) > 10) {

            //set power to zero if tics pretty high and power continually being used, stops lift
            //system from breaking itself from trying to go past mechanical max
            if((Math.abs(timeChange) > 2000)){
                liftMotorLeft.setPower(0);
                liftMotorRight.setPower(0);

                //stops while loop
                break;
            }else{
                if(Math.abs(tics) - motorPositionLeft < 0 && Math.abs(tics) - motorPositionRight < 0){
                    liftMotorLeft.setPower(power);
                    liftMotorRight.setPower(power);

                }else{
                    liftMotorLeft.setPower(0.8);
                    liftMotorRight.setPower(0.8);

                }
                motorPositionLeft = liftMotorLeft.getCurrentPosition();
                motorPositionRight = liftMotorRight.getCurrentPosition();
            }
            timeChange =  System.currentTimeMillis() - time;
        }


        liftMotorLeft.setPower(0);
        liftMotorRight.setPower(0);


    }

     */




