package org.firstinspires.ftc.teamcode.State.Common;

public class Constants {
    //TODO: Edit values
    private int heightTrueLimit = 1814;
    private int heightLimit = 1720;
    private int heightHigh = 1665;
    private int heightMed = 1225;
    private int heightLow = 728;
    private double clawOpenPos = 0.0;
    private double clawLowThreshold = 0.25;
    private double clawClosePos = 0.38; // was 0.38 for old design
    private double clawHighThreshold = 0.4;
    private int stackHeight = 210;
    private double steadyPow = 0.12;
    private int stackIntervalHeight = 50;
    private int heightBottom = 40;
    private double sleeveCamPos = 0.55;
    private double straightCamPos = 0.65;
    private double strafeMedCamPos = 0.75;
    private double strafeLowCamPos = 0.51;
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

