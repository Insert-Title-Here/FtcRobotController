package org.firstinspires.ftc.teamcode.Common;

public class Constants {
    private int heightTrueLimit = 1200;
    private int heightLimit = 1190;
    private int heightHigh = 1087;
    private int heightMed = 780;
    private int heightLow = 460;
    private double clawOpenPos = 0.0;
    private double clawClosePos = 0.36;
    private double clawLowThreshold = 0.34;
    private double clawHighThreshold = 0.38;
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

