package org.firstinspires.ftc.teamcode.Testing;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.Intake;
import org.firstinspires.ftc.teamcode.Lift;
import org.firstinspires.ftc.teamcode.MecanumDriveTrain;
import org.openftc.revextensions2.ExpansionHubEx;

import java.io.File;
import java.io.FileNotFoundException;
import java.io.PrintStream;
import java.util.HashMap;
import java.util.Map;

public class RobotK {

    public MecanumDriveTrain drive;
    public Intake intake;
    public Lift lift;

    //private final List<LynxModule> hubs;
    LynxModule hub1, hub2;
    ExpansionHubEx chub, ehub;
    //ColorRangeSensor color;
    File logFile;

    LynxModule.BulkData chubData, ehubData;
    //NormalizedRGBA rgba;
    String logString;





    public RobotK(HardwareMap hardwareMap, Telemetry telemetry) throws FileNotFoundException {
        drive = new MecanumDriveTrain(hardwareMap);
        //intake = new Intake(hardwareMap);
        lift = new Lift(hardwareMap, this, telemetry);

        hub1 = hardwareMap.get(LynxModule.class, "Control Hub");
        hub2 = hardwareMap.get(LynxModule.class, "Expansion Hub 2");

        chub = hardwareMap.get(ExpansionHubEx.class, "Control Hub");
        ehub = hardwareMap.get(ExpansionHubEx.class, "Expansion Hub 2");

        //color = hardwareMap.get(ColorRangeSensor.class, "color");

        ehub.setAllI2cBusSpeeds(ExpansionHubEx.I2cBusSpeed.FAST_400K);
        chub.setAllI2cBusSpeeds(ExpansionHubEx.I2cBusSpeed.FAST_400K);

        logFile = AppUtil.getInstance().getSettingsFile("DataReceiver.txt");


        hub1.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        hub2.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        hub1.clearBulkCache();
        hub2.clearBulkCache();

        /*
        hubs = hardwareMap.getAll(LynxModule.class);

        for(LynxModule hub: hubs){
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
            hub.clearBulkCache();
        }

         */

        chub.setLedColor(255, 255, 0);
        ehub.setLedColor(255, 0, 255);
    }

    //new Blinker.Step(Color.BLUE, 100, TimeUnit.SECONDS);


    public int getSpecificEncoderValue(int port, boolean controlHub){
        if(controlHub) {
            /*
            int encoderValue = hub1.getBulkData().getMotorCurrentPosition(port);
            hub1.clearBulkCache();
            return encoderValue;

             */
            int encoderValue = chubData.getMotorCurrentPosition(port);
            hub1.clearBulkCache();
            return encoderValue;


        }else{
            /*
            int encoderValue = hub2.getBulkData().getMotorCurrentPosition(port);
            hub2.clearBulkCache();
            return encoderValue;

             */
            int encoderValue = ehubData.getMotorCurrentPosition(port);
            hub2.clearBulkCache();
            return encoderValue;


        }
    }

    public Map<Integer, int[]> getAllEncoderValues(){
        Map<Integer, int[]> returnEncoderValues = new HashMap<>();
        int[] hub1Array = new int[4];
        int[] hub2Array = new int[4];

        LynxModule.BulkData data1 = hub1.getBulkData();
        LynxModule.BulkData data2 = hub2.getBulkData();

        for(int i = 0; i < 4; i++){
            hub1Array[i] = data1.getMotorCurrentPosition(i);
        }

        for(int i = 0; i < 4; i++){
            hub2Array[i] = data2.getMotorCurrentPosition(i);
        }

        returnEncoderValues.put(1, hub1Array);
        returnEncoderValues.put(2, hub2Array);

        return returnEncoderValues;
    }

    public void update(){
        chubData = hub1.getBulkData();
        ehubData = hub2.getBulkData();

        //rgba = color.getNormalizedColors();

        hub1.clearBulkCache();
        hub2.clearBulkCache();
    }

    public void stop(){
        drive.brake();
        //intake.clampAndRelease(false);
        //intake.brake();
    }

    public void randomColor(){
        int color1 = (int)Math.floor(Math.random() * 256);
        int color2 = (int)Math.floor(Math.random() * 256);
        int color3 = (int)Math.floor(Math.random() * 256);


        chub.setLedColor(color1, color2, color3);
        ehub.setLedColor(color2, color3, color1);
    }

    public void setColor(int r, int g, int b){
        chub.setLedColor(r, g, b);
        ehub.setLedColor(r, g, b);
    }

    public void addLog(String addition){
        logString += addition + "\n";

    }

    public void sendDataToFile(){
        try {
            PrintStream ps = new PrintStream(logFile);
            ps.println(logString);
            ps.close();
        } catch (FileNotFoundException e) {
            e.printStackTrace();
        }
    }


}
