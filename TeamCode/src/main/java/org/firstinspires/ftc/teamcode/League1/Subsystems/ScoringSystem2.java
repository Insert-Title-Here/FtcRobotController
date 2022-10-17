package org.firstinspires.ftc.teamcode.League1.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.League1.Common.Constants;

public class ScoringSystem2{
    DcMotorEx lLift, rLift;
    public Servo grabber, rLinkage, lLinkage;
    public ScoringMode height;
    private boolean grabbing, linkageUp, extended;
    Constants constants;
    private int coneStack;




    public enum ScoringMode{
        HIGH,
        MEDIUM,
        LOW,
        ULTRA
    }

    public ScoringSystem2(HardwareMap hardwareMap, Constants constants) {
        coneStack = 5;
        height = ScoringMode.HIGH;
        extended = false;
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
        grabber.setPosition(constants.open);

    }

    public int getConeStack(){
        return coneStack;
    }

    public void lowerConeStack(){
        if(coneStack - 1 > 0) {
            coneStack -= 1;
        }
    }

    public void raiseConeStack(){
        if(coneStack + 1 < 6) {
            coneStack += 1;
        }
    }

    //TODO: tune this
    public void setLinkageConeStack(){
        if(coneStack == 5){
            setLinkagePosition(0.24);
        }else if(coneStack == 4){
            setLinkagePosition(0.21);
        }else if(coneStack == 3){
            setLinkagePosition(0.18);
        }else if(coneStack == 2){
            setLinkagePosition(0.15);
        }else if(coneStack == 1){
            setLinkagePosition(0.12);
        }
    }

    public boolean isExtended() {
        return extended;
    }

    public void setExtended(boolean extended) {
        this.extended = extended;
    }

    public void setScoringMode(ScoringMode height){
        this.height = height;
    }

    public ScoringMode getScoringMode(){
        return height;
    }

    public int getHeight(){
       if(height == ScoringMode.HIGH){
           return 850;
       }else if(height == ScoringMode.MEDIUM){
           return 500;
       }else if(height == ScoringMode.LOW){
           return 190;
       }

       return 0;
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

    public void changeMode(ScoringMode score){
        height = score;
    }

    public void autoGoToPosition(){
        if(height == ScoringMode.HIGH || height == ScoringMode.ULTRA){
            moveToPosition(850, 1);

        }else if(height == ScoringMode.MEDIUM){
            moveToPosition(500, 1);


        }else if(height == ScoringMode.LOW){
            moveToPosition(190, 1);

        }

        extended = true;
    }

    public void autoGoToPosition(ScoringMode height) {
        this.height = height;
        if (height == ScoringMode.HIGH || height == ScoringMode.ULTRA) {
            moveToPosition(850, 1);

        } else if (height == ScoringMode.MEDIUM) {

            //TODO: Find tic value
            moveToPosition(500, 1);


        } else if (height == ScoringMode.LOW) {
            //TODO: Find tic value
            moveToPosition(190, 1);

        }


    }






    public void moveToPosition(int tics, double power){

        ElapsedTime time = new ElapsedTime();
        double startTime = time.seconds();

        int rLiftPos = rLift.getCurrentPosition();
        int lLiftPos = -1 * lLift.getCurrentPosition();


        if(tics < ((rLiftPos + lLiftPos) / 2)){
            power *= -1;
        }

        double rightPower = power;
        double leftPower = power;





        //Dont know if need the != condition
        //if ((tics == 0 && rLiftPos != 0 && lLiftPos != 0)) {

        //TODO: Check if logic for encoder positions works
        while ((time.seconds() - startTime) < 3 && Math.abs(rLiftPos - tics) > 20 || Math.abs(lLiftPos - tics) > 20) {

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

    //TODO: Tune this
    public void shiftLinkagePosition(){
        double setHeight = getGrabberPosition() - 0.04;

        if(setHeight < 0.09){
            setHeight = 0.25;
        }else if(setHeight > 0.25){
            setHeight = 0.25;
        }

        setLinkagePosition(setHeight);
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
            setGrabberPosition(constants.open);
        }
    }

    public void setGrabbing(boolean grabbing) {
        this.grabbing = grabbing;
    }

    public void setLinkageUp(boolean linkageUp) {
        this.linkageUp = linkageUp;
    }

    public boolean isGrabbing() {
        return grabbing;
    }

    public boolean isLinkageUp() {
        return linkageUp;
    }

    public boolean bothBusy(){
        return rLift.isBusy() && lLift.isBusy();
    }
}
