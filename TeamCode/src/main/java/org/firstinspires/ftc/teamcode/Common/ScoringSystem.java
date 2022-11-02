package org.firstinspires.ftc.teamcode.Common;

import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class ScoringSystem {
    private DcMotor liftMotor;
    private Servo claw;
    ColorRangeSensor colorCone;


    public ScoringSystem(HardwareMap hardwareMap) {
        /* the below is init*/
        claw = hardwareMap.get(Servo.class, "claw");
        liftMotor = hardwareMap.get(DcMotor.class, "motor");
        colorCone = hardwareMap.get(ColorRangeSensor.class, "colorCone");
        // reset encoder's tics for liftMotor (leave commented unless you need to reset the encoder for
        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        //Not actually without encoder (just doesn't use given PID)
        liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // when the power is zero, it'll resist movement/change
        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


    }

    //goes to given tics
    public void goToPosition(int tics, double power) {

        int motorPosition = liftMotor.getCurrentPosition();

        if (motorPosition > tics) {
            power *= -1;
        }
        long time = System.currentTimeMillis();
        long difference;
        while (Math.abs(Math.abs(tics)-motorPosition) > 10) {
            //set power to zero if tics pretty high and power continually being used
            /*
            if(motorPosition < 400){
                liftMotor.setPower(power/2);
                motorPosition = liftMotor.getCurrentPosition();
            }else{
                liftMotor.setPower(power);
                motorPosition = liftMotor.getCurrentPosition();
            }

             */



            difference =  System.currentTimeMillis() - time;
            //set power to zero if tics pretty high and power continually being used
            if((Math.abs(difference) > 5000)){
                liftMotor.setPower(0);

            }else{
                liftMotor.setPower(power);
                motorPosition = liftMotor.getCurrentPosition();
            }





        }


        liftMotor.setPower(0);

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

    public int currentBlueColor() {
        return colorCone.blue(); // if current color is really high // 410
    }

    public int currentRedColor() {
        return colorCone.red(); // if current color is really high // 177
    }

    public boolean grabCone() {
        if (colorCone.getDistance(DistanceUnit.CM) < 0.3) {
            // grab cone
            setClawPosition(0.24);
            // lift up
            goToPosition(600, 0.6);

            return true;

        }
        return false;
    }
}




