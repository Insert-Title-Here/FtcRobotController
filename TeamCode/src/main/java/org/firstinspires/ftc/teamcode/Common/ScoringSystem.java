package org.firstinspires.ftc.teamcode.Common;

import static java.lang.Thread.sleep;

import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class ScoringSystem {
    private DcMotor liftMotor;
    private Servo claw;
    ColorRangeSensor colorCone;
    Telemetry telemetry;

    public ScoringSystem(HardwareMap hardwareMap, Telemetry telemetry) {
        /* the below is init*/
        claw = hardwareMap.get(Servo.class, "claw");
        liftMotor = hardwareMap.get(DcMotor.class, "motor");
        colorCone = hardwareMap.get(ColorRangeSensor.class, "colorCone");
        this.telemetry = telemetry;
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
        boolean notReached = true;
        while (Math.abs(Math.abs(tics)-motorPosition) > 10 && notReached) {
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
            //set power to zero if tics pretty high and power continually being used, stops lift
            //system from breaking itself from trying to go past mechanical max
            if((Math.abs(difference) > 5000)){
                liftMotor.setPower(0);
                notReached = false;
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
            setClawPosition(0.24);
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
    public boolean grabCone(boolean stack) throws InterruptedException {
        if (colorCone.getDistance(DistanceUnit.CM) < 1) {
            // grab cone
            setClawPosition(0.24);
            sleep(400);
            // lift up
            if(stack){
                goToPosition(getEncoderPosition() + 320, 0.6);
            }else{
                goToPosition(getEncoderPosition() + 100, 0.6);
            }
            telemetry.addData("distance", colorCone.getDistance(DistanceUnit.CM));
            telemetry.update();
            return true;

        }
        return false;
    }



    public double getDistance() {
        return colorCone.getDistance(DistanceUnit.CM);
    }
}




