package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Lift {

    public DcMotor lift;

    public Lift(HardwareMap hardwareMap){
        lift = hardwareMap.get(DcMotor.class, "lift");
    }



}
