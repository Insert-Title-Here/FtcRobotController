package org.firstinspires.ftc.teamcode.Common;

import com.acmerobotics.dashboard.config.Config;

@Config
public class Constants {

    // Linkage positions
    public static double LINKAGE_DOWN = 0.2;
    public static double LINKAGE_UP = 0.92;
    public static double LINKAGE_INTAKE = 0.17;

    // Intake linkage positions
    public static double INTAKE_LINKAGE_DOWN = 0.04;
    public static double INTAKE_LINKAGE_UP = 0.5;

    //Grabber Positions
    public static double GRABBING = 0.15;
    public static double OPEN = 0.35;

    //Drive Modifiers
    public static double NORMAL_LINEAR_MODIFIER = 0.5;
    public static double NORMAL_ROTATIONAL_MODIFIER = 0.4;
    public static double EXTENDED_LINEAR_MODIFIER = 0.5;
    public static double EXTENDED_ROTATIONAL_MODIFIER = 0.3;
    public static double SPRINT_LINEAR_MODIFIER = 1;
    public static double SPRINT_ROTATIONAL_MODIFIER = 1;

    // Lift Heights
    public static int LIFT_LOW = 145;
    public static int LIFT_MEDIUM = 350;
    public static int LIFT_HIGH = 450;

    // Lift PID
    public static double LIFT_P = 0.001;
    public static double LIFT_I = 0;
    public static double LIFT_D = 0;
    public static double LIFT_F = 0.2;

    // Climb Target
    public static int CLIMB_HEIGHT = 2400;

}