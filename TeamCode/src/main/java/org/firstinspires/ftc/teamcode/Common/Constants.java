package org.firstinspires.ftc.teamcode.Common;

public class Constants {
    private int heightTrueLimit = 1370;
    private int heightLimit = 1367;
    private int heightHigh = 1350;
    private int heightMed = 975;
    private int heightLow = 580;
    private double clawOpenPos = 0.0;
    private double clawClosePos = 0.31;
    private double clawLowThreshold = 0.28;
    private double clawHighThreshold = 0.35;
    private int stackHeight = 174;
    private double steadyPow = 0.16;
    private int stackIntervalHeight = 50;
    private int heightBottom = 10;
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

}

