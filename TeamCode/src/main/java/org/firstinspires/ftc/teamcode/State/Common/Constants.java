package org.firstinspires.ftc.teamcode.State.Common;

public class  Constants {
    //TODO: Edit values
    private int heightTrueLimit = 1378;
    private int heightLimit = 1358;
    private int heightHigh = 1195;
    private int heightMed = 835;
    private int heightLow = 460;
    private double clawOpenPos = 0.0;
    private double clawLowThreshold = 0.25;
    private double clawClosePos = 0.36; // was 0.38 for old design
    private double clawHighThreshold = 0.8;
    private int stackHeight = 188;
    private double steadyPow = 0.18;
    private int stackIntervalHeight = 40;
    private int heightBottom = 0;
    private double sleeveCamPos = 0.23;
    private double straightCamPos = 0.32;
    private double strafeMedCamPos = 0.75;
    private double strafeLowCamPos = 0.21;
    //TODO: add comments
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
}

