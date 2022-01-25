package org.firstinspires.ftc.teamcode.Common;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.MecanumCode.Common.Constants;

public class CartesianDriveTrain {

    public DcMotor fl, fr, bl, br;
    private DcMotor fle, fre, ble, bre;

    Constants constants = new Constants();
    private BNO055IMU imu;


    public CartesianDriveTrain(HardwareMap hardwareMap){
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";

        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);



        fl = hardwareMap.dcMotor.get("FrontLeftDrive");
        fr = hardwareMap.dcMotor.get("FrontLeftDrive");
        bl = hardwareMap.dcMotor.get("FrontLeftDrive");
        br = hardwareMap.dcMotor.get("FrontLeftDrive");

        fle = hardwareMap.dcMotor.get("");
        fre = hardwareMap.dcMotor.get("");
        ble = hardwareMap.dcMotor.get("");
        bre = hardwareMap.dcMotor.get("");

        correctMotors();


    }

    //Direct to target by turning and going to it (angle is in radians?)
    //TODO: test out the directDrive
    public void directDrive(int frontBack, int rightLeft, double power){

        double angle;
        if(getSign(frontBack) == -1){

            angle = 180 - Math.abs(Math.atan(rightLeft/frontBack)) + imu.getAngularOrientation().firstAngle;

            if(getSign(rightLeft) == -1){
                angle *= -1;
            }
        }else{
            angle = Math.atan(rightLeft/frontBack) + imu.getAngularOrientation().firstAngle;

        }

        power *= getSign(angle);

        while(Math.abs(imu.getAngularOrientation().firstAngle - angle) < 0.05){
            setPower(power, -power, power, -power);
        }


        int driveDistance = (int)(Constants.TICS_TO_FOOT * Math.sqrt((rightLeft * rightLeft) + (frontBack * frontBack)));

        /*
        Can also test:

        int driveDistance = (int)(Constants.TICS_TO_FOOT * (rightLeft / Math.sin(angle));
         */

        while(fle.getCurrentPosition() < driveDistance && fre.getCurrentPosition() < driveDistance
                && ble.getCurrentPosition() < driveDistance && bre.getCurrentPosition() < driveDistance){
            setPower(power, power, power, power);
        }

        angle *= -1;
        power *= getSign(angle);

        while(Math.abs(imu.getAngularOrientation().firstAngle - angle) < 0.05){
            setPower(power, -power, power, -power);
        }




    }


    private int getSign(double num){
        if(num < 0){
            return -1;
        }else {
            return 1;
        }
    }

    public void setPower(double flPow, double frPow, double blPow, double brPow) {
        fl.setPower(-flPow);
        fr.setPower(-frPow) ;
        bl.setPower(blPow);
        br.setPower(brPow);
    }


    private void correctMotors() {
        fl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }

    private void brake() {
        fl.setPower(0);
        fr.setPower(0);
        bl.setPower(0);
        br.setPower(0);
    }

}
