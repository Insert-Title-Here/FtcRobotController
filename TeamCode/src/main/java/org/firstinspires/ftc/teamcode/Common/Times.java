package org.firstinspires.ftc.teamcode.Common;

public class Times {
    public long initializedTime;
    public static long startingTime = 0;
    public Times(){
        initializedTime = System.currentTimeMillis();
    }
    public static void startTime(){
        startingTime = System.currentTimeMillis();
    }

    public static long timeDifference(){
        long time = startingTime;
        startingTime = 0;
        return (System.currentTimeMillis() - time)/1000;
    }

}
