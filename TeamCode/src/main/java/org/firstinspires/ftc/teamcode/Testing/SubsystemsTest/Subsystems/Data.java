package org.firstinspires.ftc.teamcode.Testing.SubsystemsTest.Subsystems;

import android.graphics.Color;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;


public class Data extends Thread {
    private volatile LinearOpMode opMode;
    private volatile LynxModule chub, ehub;
    private volatile BNO055IMU imu;
    private volatile ColorRangeSensor distance;

    //Could try out Encoder class or DcMotorImpl/DcMotorImplEx or DcMotorController/DcMotorControllerEx
    //DcMotorImplEx seems to have the most functionality


    LynxModule.BulkData chubData, ehubData;
    private volatile boolean collectData, update;
    private final long loopTime = 20;
    private String howLoopingIsGoing = "Start";
    private Orientation imuAngle;
    private NormalizedRGBA rgba;



    public Data(HardwareMap hardwareMap, ColorRangeSensor distance){
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.RADIANS;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        this.distance = distance;

        chub = hardwareMap.get(LynxModule.class, "Control Hub");

        //TODO: see if the deviceName is correct
        ehub = hardwareMap.get(LynxModule.class, "Expansion Hub 1");

        chub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        ehub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);

        chub.clearBulkCache();
        ehub.clearBulkCache();

        update = true;
        collectData = true;


    }

    public void setUpdate(boolean update){
        this.update = update;
    }

    public boolean getUpdate(){
        return update;
    }

    public LynxModule.BulkData getChubData(){
        return chubData;
    }

    public LynxModule.BulkData getEhubData(){
        return ehubData;
    }

    public Orientation getAngle(){
        return imuAngle;
    }

    public NormalizedRGBA getColor(){
        return rgba;
    }

    public String getHowLoopingIsGoing(){
        return howLoopingIsGoing;
    }


    private void update(){

        chubData = chub.getBulkData();
        ehubData = ehub.getBulkData();
        imuAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
        rgba = distance.getNormalizedColors();

        chub.clearBulkCache();
        ehub.clearBulkCache();

    }




    @Override
    public void run() {

        while(collectData){
            if(update){
                long startLoop = System.currentTimeMillis();
                update();

                long timeDifference = System.currentTimeMillis() - startLoop;
                if(timeDifference < loopTime){
                    try {
                        sleep(loopTime - timeDifference);
                    } catch (InterruptedException e) {
                        throw new RuntimeException(e);
                    }

                    howLoopingIsGoing = "Loop Time is Less";
                }else{
                    howLoopingIsGoing = "Loop Time is Greater";

                }
            }

        }


        imu.close();
        chub.clearBulkCache();
        ehub.clearBulkCache();
    }


    public void stopThread(){
        collectData = false;
    }
}
