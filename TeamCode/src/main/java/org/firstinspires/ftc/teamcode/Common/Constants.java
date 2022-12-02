package org.firstinspires.ftc.teamcode.Common;

public class Constants {
    //TODO: Edit values
    private int heightTrueLimit = 1680;
    private int heightLimit = 1675;
    private int heightHigh = 1655;
    private int heightMed = 1337;
    private int heightLow = 738;
    private double clawOpenPos = 0.0;
    private double clawLowThreshold = 0.3;
    private double clawClosePos = 0.38; // was 0.38 for old design
    private double clawHighThreshold = 0.4;
    private int stackHeight = 240;
    private double steadyPow = 0.16;
    private int stackIntervalHeight = 60;
    private int heightBottom = 5;
    private double sleeveCamPos = 0.4;
    private double straightCamPos = 0.5;
    private double strafeCamPos = 0.6;
    //TODO: add comments
    public double getClawOpenPos(){
        return clawOpenPos;
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
    public double getStrafeCamPos() {
        return strafeCamPos;
    }
    public double getStraightCamPos() {
        return straightCamPos;
    }
}

