package org.firstinspires.ftc.teamcode.Testing.SubsystemsTest.Subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;


public class Robot {
    Data data;
    MecDriveBulk drive;
    ScoringSystemBulk score;

    public Robot(HardwareMap hardwareMap){
        data = new Data(hardwareMap);
        drive = new MecDriveBulk(hardwareMap, data);
        score = new ScoringSystemBulk(hardwareMap, data, true);
    }
}
