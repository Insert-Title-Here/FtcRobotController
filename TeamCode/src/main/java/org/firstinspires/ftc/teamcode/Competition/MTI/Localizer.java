package org.firstinspires.ftc.teamcode.Competition.MTI;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.roadrunnerfiles.util.Encoder;

public class Localizer {
    Encoder leftEncoder, rightEncoder, middleEncoder;

    public Localizer(HardwareMap hardwareMap){
        leftEncoder = hardwareMap.get(Encoder.class, "name");
        rightEncoder = hardwareMap.get(Encoder.class, "name");
        middleEncoder = hardwareMap.get(Encoder.class, "name");


    }
}
