package org.firstinspires.ftc.teamcode.Auto;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Common.MecanumDrive;
import org.firstinspires.ftc.teamcode.Common.ScoringSystem;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicInteger;

//TODO:Needs testing
@Autonomous
public class PIDTest extends LinearOpMode {
    MecanumDrive drive;
    ScoringSystem score;
    AtomicInteger position;
    Thread liftThread;
    BNO055IMU imu;

    @Override
    public void runOpMode() throws InterruptedException {
        //initializing imu and camera
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // seehe calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";

        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        drive = new MecanumDrive(hardwareMap, telemetry);
        score = new ScoringSystem(hardwareMap, telemetry);
        position = new AtomicInteger();


        //thread for slides
        liftThread = new Thread(){
            @Override
            public void run(){
                // keeps the slides from sliding down on its own
                while(opModeIsActive()){
                    telemetry.addData("flPos", drive.getFLPosition());
                    telemetry.addData("frPos", drive.getFRPosition());
                    telemetry.addData("blPos", drive.getBLPosition());
                    telemetry.addData("brPos", drive.getBRPosition());
                    telemetry.addData("expectedPos", position.get());
                    telemetry.addData("Ppower", drive.getProportionPower());
                    telemetry.addData("Ipower", drive.getIntegralPower());
                    telemetry.addData("Dpower", drive.getDerivativePower());
                    telemetry.addData("drivePower",(drive.getProportionPower()+drive.getIntegralPower()+drive.getDerivativePower()));
                    telemetry.update();
                }

            }
        };
        //TRY SETTING THE COMMANDS INSIDE THE THREAD AND SEE IF IT WORKS THAT WAY

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();
        liftThread.start();

        PIDTests();
    }
    public void PIDTests(){
        //drive.goToPosition(0.3, 0.3,  0.3, 0.3, avgPosition(2307, 2203, 2230, 2313), "forward");
    }
    public int avgPosition(int fl, int fr, int bl, int br){
        return (int)(Math.abs(fl) + Math.abs(fr) + Math.abs(bl) + Math.abs(br))/4;
    }


}
