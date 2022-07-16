package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.openftc.revextensions2.ExpansionHubEx;

import java.util.concurrent.atomic.AtomicBoolean;

public class Robot {

    private volatile LynxModule.BulkData ehData, chData;
    private volatile NormalizedRGBA rgba;

    private LynxModule ch, eh;
    private ExpansionHubEx ehub, chub;
    private BNO055IMU imu;




    ColorRangeSensor color;
    public synchronized void update(){
        chData = ch.getBulkData();
        ehData = eh.getBulkData();

        rgba = color.getNormalizedColors();

        eh.clearBulkCache();
        ch.clearBulkCache();
    }


    //Control Constructor
    public Robot(HardwareMap hardwareMap){
        eh = hardwareMap.get(LynxModule.class, "Expansion Hub");
        ch = hardwareMap.get(LynxModule.class, "Control Hub");

        ehub  = hardwareMap.get(ExpansionHubEx.class, "Expansion Hub");
        chub  = hardwareMap.get(ExpansionHubEx.class, "Control Hub");

        ehub.setAllI2cBusSpeeds(ExpansionHubEx.I2cBusSpeed.FAST_400K);
        chub.setAllI2cBusSpeeds(ExpansionHubEx.I2cBusSpeed.FAST_400K);

        color = hardwareMap.get(ColorRangeSensor.class, "Color");


        eh.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        ch.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        eh.clearBulkCache();
        ch.clearBulkCache();

    }


    public int getMotorPos(int port, boolean chub){
        if(chub){
            return chData.getMotorCurrentPosition(port);
        }else{
            return ehData.getMotorCurrentPosition(port);
        }
    }
}
