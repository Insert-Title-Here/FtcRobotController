package org.firstinspires.ftc.teamcode.League1.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.League1.Common.Constants;

public class ScoringSystem2 {
    DcMotorEx lLift, rLift;
    public Servo grabber, rLinkage, lLinkage;
    Constants constants;

    public ScoringSystem2(HardwareMap hardwareMap, Constants constants) {
        this.constants = constants;

        rLift = hardwareMap.get(DcMotorEx.class, "RightLift");
        lLift = hardwareMap.get(DcMotorEx.class, "LeftLift");

        rLift.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        lLift.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);


        rLift.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        lLift.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        rLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        lLinkage = hardwareMap.get(Servo.class, "LeftLinkage");
        rLinkage = hardwareMap.get(Servo.class, "RightLinkage");

        grabber = hardwareMap.get(Servo.class, "Grabber");


        lLinkage.setPosition(0.03);
        rLinkage.setPosition(0.03);
        grabber.setPosition(constants.openAuto);

    }
    public void setPower(double power) {
        rLift.setPower(power);
        lLift.setPower(-power);
    }

    public void setPower(double rightPower, double leftPower){
        rLift.setPower(rightPower);
        lLift.setPower(-leftPower);
    }
    public int getLeftEncoderPos() {
        return lLift.getCurrentPosition();
    }

    public int getRightEncoderPos() {
        return rLift.getCurrentPosition();
    }



    public void moveToPosition(int tics, double power){

        int rLiftPos = rLift.getCurrentPosition();
        int lLiftPos = -1 * lLift.getCurrentPosition();


        if(tics < /*(*/rLiftPos/* + lLiftPos) / 2*/){
            power *= -1;
        }

        double rightPower = power;
        double leftPower = power;





        //Dont know if need the != condition
        //if ((tics == 0 && rLiftPos != 0 && lLiftPos != 0)) {

        //TODO: Check if logic for encoder positions works
        while (Math.abs(rLiftPos - tics) > 20 || Math.abs(lLiftPos - tics) > 20) {

            //TODO: figure out if we need to negate either of them

            if(!(Math.abs(rLiftPos - tics) > 20)){
                rightPower = 0;
            }else if(!(Math.abs(lLiftPos - tics) > 20)){
                leftPower = 0;
            }


            rLiftPos = rLift.getCurrentPosition();
            lLiftPos = -1 * lLift.getCurrentPosition();

            setPower(rightPower, leftPower);




        }

        setPower(0);

    }

    public void setLinkagePosition(double position){
        //TODO: tune position values
        rLinkage.setPosition(position);
        lLinkage.setPosition(position);
    }

    public double getRightLinkage(){
        return rLinkage.getPosition();
    }

    public double getLeftLinkage(){
        return lLinkage.getPosition();
    }

    public void linkageAutomated(boolean up){
        if(up){
            setLinkagePosition(constants.linkageUp);
        }else{
            setLinkagePosition(constants.linkageDown);

        }
    }

    public void linkageUpAndDown(boolean up){
        if(up) {
            setLinkagePosition(constants.linkageUp);
        }else{
            setLinkagePosition(constants.linkageDown);
        }
    }

    public int getEncoderPosition(boolean right){
        if(right){
            return rLift.getCurrentPosition();
        }else{
            return lLift.getCurrentPosition();

        }
    }

    public void reset(){
        lLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        lLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }

    public boolean isBusy(){
        return rLift.isBusy() && lLift.isBusy();
    }




    public void setGrabberPosition(double position){
        grabber.setPosition(position);
    }

    public double getGrabberPosition(){
        return grabber.getPosition();
    }


    public void grabberOpenAndClose(boolean close){
        if(close) {
            setGrabberPosition(constants.grabbing);
        }else{
            setGrabberPosition(constants.openAuto);
        }
    }

    public boolean bothBusy(){
        return rLift.isBusy() && lLift.isBusy();
    }
}
