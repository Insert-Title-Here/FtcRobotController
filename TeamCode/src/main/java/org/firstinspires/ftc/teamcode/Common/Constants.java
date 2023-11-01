package org.firstinspires.ftc.teamcode.Common;

import com.acmerobotics.dashboard.config.Config;

@Config
public class Constants {

    // Linkage positions
    public static double LINKAGE_DOWN = 0.2;
    public static double LINKAGE_UP = 0.96;
    public static double LINKAGE_INTAKE = 0.18;

    // Intake linkage positions
    public static double INTAKE_LINKAGE_DOWN = 0.2;
    public static double INTAKE_LINKAGE_UP = 0.55;
    public static double INTAKE_SPEED = 0.35;

    //Grabber Positions
    public static double GRABBING = 0.07;
    public static double OPEN = 0.5;

    //Drive Modifiers
    public static double NORMAL_LINEAR_MODIFIER = 0.25;
    public static double NORMAL_ROTATIONAL_MODIFIER = 0.4;
    public static double EXTENDED_LINEAR_MODIFIER = 0.4;
    public static double EXTENDED_ROTATIONAL_MODIFIER = 0.3;
    public static double SPRINT_LINEAR_MODIFIER = 1;
    public static double SPRINT_ROTATIONAL_MODIFIER = 1;

    //Auto Drive speeds
    public static double AUTO_LINEAR_SPEED = 0.4;
    public static double AUTO_ROTATIONAL_SPEED = 0.3;
    public static double AUTO_SLOWED_SPEED = -0.1;
    public static double AUTO_SAFE_MO = 0.2;

    public static double AUTO_SUPER_MO = 0.6;

    public static double AUTO_SUPER_TURN = 0.5;

    // Lift Heights
    public static int LIFT_DOWN=0;
    public static int LIFT_LOW = 145;
    public static int LIFT_MEDIUM = 450;
    public static int LIFT_HIGH = 750;

    // Lift PID
    public static double LIFT_P = 0.001;
    public static double LIFT_I = 0;
    public static double LIFT_D = 0;
    public static double LIFT_F = 0.2;

    // Climb Target
    public static int CLIMB_HEIGHT = 2400;

    //airplaneeeey
    public static double AIRPLANE_START_POS = 0.0;
    public static double AIRPLANE_RELEASE_POS = 0.8;

    //bumper/auto score device
    public static double AUTO_SCORING_CLAMP_OPEN = 0;
    public static double AUTO_SCORING_CLAMP_CLOSED = 0.7;

}