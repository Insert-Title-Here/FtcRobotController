package org.firstinspires.ftc.teamcode.Common;

import static java.lang.Thread.currentThread;
import static java.lang.Thread.sleep;

import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
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
    MecanumDrive drive;
    ColorRangeSensor colorCone;
    Telemetry telemetry;
    Constants constant;
    File loggingFile = AppUtil.getInstance().getSettingsFile("telemetry.txt");

    public String loggingString;

    public ScoringSystem(HardwareMap hardwareMap, Telemetry telemetry) {
        /* the below is init*/
        claw = hardwareMap.get(Servo.class, "claw");
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
        int negPos = 1;
        if (motorPosition > tics) {
            negPos *= -1;
        }
        double startingPos = getEncoderPosition();
        long time = System.currentTimeMillis();
        long timeChange= 0;
        boolean notReached = true;
        double currentError = tics - getEncoderPosition();
        double priorError = tics - getEncoderPosition();
        double proportionPow;
        double derivativePow;
        while (Math.abs(Math.abs(tics)-motorPosition) > 1 && notReached) {
            if(tics == 0){
                proportionPow = currentError * 0.004;
                derivativePow = /*((currentError-priorError)/timeChange) * 2*/0.2;
            }else{
                proportionPow = /*currentError * 0.82*/1;
                derivativePow =  0/*((currentError-priorError)/timeChange) * 0.65*/;
            }
            priorError = currentError;

            //set power to zero if tics pretty high and power continually being used, stops lift
            //system from breaking itself from trying to go past mechanical max
            if((Math.abs(timeChange) > 3000)){
                liftMotor.setPower(0);
                //stops while loop
                notReached = false;
                break;
            }else{
                if(startingPos < 300 && tics == 0){
                    liftMotor.setPower(negPos*0.1);

                }else{
                    liftMotor.setPower(proportionPow + derivativePow);
                    motorPosition = liftMotor.getCurrentPosition();
                }
            }
            currentError = tics - getEncoderPosition();
            timeChange =  System.currentTimeMillis() - time;
            loggingString += "PriorError: " + priorError + "   ";
            loggingString += "CurrentError: " + currentError + "   ";
            loggingString += "proportionPow: " + proportionPow + "   ";
            loggingString += "derivativePower: " + derivativePow + "   ";
            loggingString += "CurrentPower: " + getPower() + "   ";
            loggingString += "DrivePower: " + (proportionPow + derivativePow) + "\n";
        }
        loggingString += "---------------------------------------------------\n";


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

    public int getEncoderPosition() {
        return liftMotor.getCurrentPosition();
    }

    public double getClawPosition() {
        return claw.getPosition();
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
            goToPosition(getEncoderPosition() + 100, 0.6);
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
                goToPosition(getEncoderPosition() + 50, 0.6);
            }else{
                goToPosition(getEncoderPosition() + 100, 0.6);
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
            goToPosition(getEncoderPosition() + 50, 0.6);

            telemetry.addData("distance", colorCone.getDistance(DistanceUnit.CM));
            telemetry.update();
        }
    }

    public double getDistance() {
        return colorCone.getDistance(DistanceUnit.CM);
    }
}




