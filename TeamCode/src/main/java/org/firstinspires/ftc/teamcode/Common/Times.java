package org.firstinspires.ftc.teamcode.Common;

import java.util.Timer;

public class Times {
    public long initializedTime;
    public static long startingTime;
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
