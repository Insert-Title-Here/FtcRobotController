package org.firstinspires.ftc.teamcode.Common;

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
    private DcMotor liftMotor;
    private Servo claw;
    private Servo camTurn;
    MecanumDrive drive;
    ColorRangeSensor colorCone;
    Telemetry telemetry;
    Constants constant;
    File loggingFile = AppUtil.getInstance().getSettingsFile("telemetry.txt");

    public String loggingString;

    public ScoringSystem(HardwareMap hardwareMap, Telemetry telemetry) {
        /* the below is init*/
        constant = new Constants();
        claw = hardwareMap.get(Servo.class, "claw");
        camTurn = hardwareMap.get(Servo.class, "camTurn");
        liftMotor = hardwareMap.get(DcMotor.class, "motor");
        colorCone = hardwareMap.get(ColorRangeSensor.class, "colorCone");
        drive = new MecanumDrive(hardwareMap, telemetry);
        this.telemetry = telemetry;
        // reset encoder's tics for liftMotor (leave commented unless you need to reset the encoder for
        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        //Not actually without encoder (just doesn't use given PID)
        liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // when the power is zero, it'll resist movement/change
        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        liftMotor.setDirection(DcMotor.Direction.REVERSE);

    }

    //goes to given tics
    public void goToPosition(int tics, double power) {
        int motorPosition = liftMotor.getCurrentPosition();
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
                liftMotor.setPower(0);
                //stops while loop
                break;
            }else{
                if(Math.abs(tics) - motorPosition < 0){
                    liftMotor.setPower(power);
                }else{
                    liftMotor.setPower(0.8);
                }
                motorPosition = liftMotor.getCurrentPosition();
            }
            timeChange =  System.currentTimeMillis() - time;
        }


        liftMotor.setPower(0);

    }

    public void writeLoggerToFile(){
        try{
            PrintStream toFile = new PrintStream(loggingFile);
            toFile.println(loggingString);
        }catch(FileNotFoundException e){
            e.printStackTrace();
        }
    }
    public void setPower(double power){
        liftMotor.setPower(power);
    }
    public double getPower(){
        return liftMotor.getPower();
    }

    public void setClawPosition(double position) {
        claw.setPosition(position);
    }

    public void setCamPosition(double position) {
        camTurn.setPosition(position);
    }

    public int getEncoderPosition() {
        return liftMotor.getCurrentPosition();
    }

    public double getClawPosition() {
        return claw.getPosition();
    }
    public double getCamPosition() {
        return camTurn.getPosition();
    }

    public void resetLiftEncoder(){
         liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public boolean isBusy(){
        return liftMotor.isBusy();
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
        if (colorCone.getDistance(DistanceUnit.CM) < 0.9) {
            // grab cone
            setClawPosition(constant.getClawClosePos());
            sleep(400);
            // lift up
            goToPosition(getEncoderPosition() + 100, 1);
            telemetry.addData("distance", colorCone.getDistance(DistanceUnit.CM));
            telemetry.update();
            return true;

        }
        return false;
    }
    //uses color sensor to grab cone(this one is used when trying to grab from the stack of 5 cones
    // tele version
    public boolean grabCone(boolean stack) throws InterruptedException {
        if (colorCone.getDistance(DistanceUnit.CM) < 1) {
            // grab cone
            setClawPosition(constant.getClawClosePos());
            sleep(400);
            // lift up
            if(stack){
                sleep(300);
                goToPosition(getEncoderPosition() + 330, 1);
            }else{
                goToPosition(getEncoderPosition() + 100, 1);
            }
            telemetry.addData("distance", colorCone.getDistance(DistanceUnit.CM));
            telemetry.update();
            return true;

        }
        return false;
    }
    // auto version
    public void grabConeAuto() {
        drive.goToPosition(0.5, 0.5, 0.5, 0.5);
        if (colorCone.getDistance(DistanceUnit.CM) < 1) {

            // grab cone
            setClawPosition(constant.getClawClosePos());
            try {
                sleep(400);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
            // lift up
            goToPosition(getEncoderPosition() + 50, 1);

            telemetry.addData("distance", colorCone.getDistance(DistanceUnit.CM));
            telemetry.update();
        }
    }

    public double getDistance() {
        return colorCone.getDistance(DistanceUnit.CM);
    }
    /*
    //TODO Test the actual heights
    int height1 = 174;
    int height2 = 124;
    int height3 = 74;
    int height4 = 24;
    int height5 = 24;
    int currentHeight = 174;
    public void stackUp(){
        if(getEncoderPosition() >= height1){
            currentHeight = height1;
        }else if(getEncoderPosition() >= height2){
            currentHeight = height2;
        }else if(){
            currentHeight = height3;
        }else if(){
            currentHeight = height4;
        }else{
            currentHeight = height5;
        }
    }
    public void stackDown(){
        if(){
            currentHeight = height1;
        }else if(){
            currentHeight = height2;
        }else if(){
            currentHeight = height3;
        }else if(){
            currentHeight = height4;
        }else{
            currentHeight = height5;
        }
    }

     */
}




