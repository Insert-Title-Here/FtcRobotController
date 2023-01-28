package org.firstinspires.ftc.teamcode.State.Common;

public class  Constants {
    //TODO: Edit values
    private int heightTrueLimit = 1378;
    private int heightLimit = 1358;
    private int heightHigh = 1190;
    private int heightMed = 870;
    private int heightLow = 490;
    private double clawOpenPos = 0.0;
    private double clawLowThreshold = 0.2;
    private double clawClosePos = 0.35; // was 0.38 for old design



    private double autoClawClosePos = 0.37;
    private double clawHighThreshold = 0.5;
    private int stackHeight = 188;
    private double steadyPow = 0.18;
    private int stackIntervalHeight = 50;


    private int stack4 = 129;
    private int stack3 = 79;
    private int stack2 = 33;



    private int heightBottom = 0;
    private double sleeveCamPos = 0.23;
    private double straightCamPos = 0.32;
    private double strafeMedCamPos = 0.75;
    private double strafeLowCamPos = 0.21;
    private double strafeConeCamPos = 0.22;
    //TODO: add comments
    public double getAutoClawClosePos() {
        return autoClawClosePos;
    }

    public int getStack4() {
        return stack4;
    }

    public int getStack2() {
        return stack2;
    }

    public int getStack3() {
        return stack3;
    }

    public double getClawOpenPos(){
        return clawOpenPos;
    }
    public void setClawClosePos(double val){
        clawClosePos = val;
    }
    public double getClawClosePos(){
        return clawClosePos;
    }
    public double getClawLowThreshold(){
        return clawLowThreshold;
    }
    public double getClawHighThreshold(){
        return clawHighThreshold;
    }
    public int getHeightTrueLimit(){
        return heightTrueLimit;
    }
    public int getHeightLimit(){
        return heightLimit;
    }
    public int getHeightHigh(){
        return heightHigh;
    }
    public int getHeightMed(){
        return heightMed;
    }
    public int getHeightLow(){
        return heightLow;
    }
    public int getStackHeight(){
        return stackHeight;
    }
    public double getSteadyPow(){
        return steadyPow;
    }
    public int getStackIntervalHeight(){
        return stackIntervalHeight;
    }
    public int getHeightBottom(){
        return heightBottom;
    }
    public void setHeightBottom(int tics){
        heightBottom = tics;
    }
    public double getSleeveCamPos() {
        return sleeveCamPos;
    }
    public double getStrafeMedCamPos() {
        return strafeMedCamPos;
    }
    public double getStraightCamPos() {
        return straightCamPos;
    }

    public double getStrafeLowCamPos() {
        return strafeLowCamPos;
    }

    public double getStrafeConeCamPos() {
        return strafeConeCamPos;
    }

    public enum Pipeline {
        PARK,
        POLE,
        CONE
    }
}

