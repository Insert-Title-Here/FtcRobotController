package org.firstinspires.ftc.teamcode.Testing.SubsystemsTest.Subsystems;

import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;


public class Robot {
    public Data data;
    public MecDriveBulk drive;
    public ScoringSystemBulk score;
    private ColorRangeSensor distance;

    private volatile boolean optionsFlag;


    public Robot(HardwareMap hardwareMap){
        drive = new MecDriveBulk(hardwareMap, data);
        score = new ScoringSystemBulk(hardwareMap, data, true);
        distance = hardwareMap.get(ColorRangeSensor.class, "distance");
        distance.setGain(250);
        data = new Data(hardwareMap, distance);

        setOptionsFlag(true);

    }

    //TODO: see if we want the setUpdate stuff (is opMode looptime too fast)
    public NormalizedRGBA getColor(){
        //data.setUpdate(false);
        NormalizedRGBA rgba = data.getColor();
        //data.setUpdate(true);

        return rgba;
    }

    public void setOptionsFlag(boolean optionsFlag){
        this.optionsFlag = optionsFlag;
    }

    public boolean getOptionsFlag(){
        return optionsFlag;
    }

}
