package org.firstinspires.ftc.teamcode.Testing.Demo;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class DriveTrainYay {

    DcMotor fl;
    DcMotor fr;
    DcMotor bl;
    DcMotor br;
    BNO055IMU imu;
    // constructor
    public DriveTrainYay (HardwareMap hardwareMap) {
        fl = hardwareMap.get(DcMotor.class, "FrontLeftDrive");
        fr = hardwareMap.get(DcMotor.class, "FrontRightDrive");
        bl = hardwareMap.get(DcMotor.class, "BackLeftDrive");
        br = hardwareMap.get(DcMotor.class, "BackRightDrive");

        fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        br.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        fl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        fl.setDirection(DcMotorSimple.Direction.FORWARD);
        fr.setDirection(DcMotorSimple.Direction.REVERSE);
        bl.setDirection(DcMotorSimple.Direction.FORWARD);
        br.setDirection(DcMotorSimple.Direction.REVERSE);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";

        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);



    }

    public void setPower(double flPower, double frPower, double blPower, double brPower) {
        fl.setPower(flPower);
        fr.setPower(frPower);
        bl.setPower(blPower);
        br.setPower(brPower);
    }
    public void setPowerStraight(double power) {
        fl.setPower(power);
        fr.setPower(power);
        bl.setPower(power);
        br.setPower(power);
    }

    public double getFLMotorPosition() {
        return fl.getCurrentPosition();
    }

    public double getFRMotorPosition() {
        return fr.getCurrentPosition();
    }

    public double getBLMotorPosition() {
        return bl.getCurrentPosition();
    }

    public double getBRMotorPosition() {
        return br.getCurrentPosition();
    }

    public void bangBangController(double targetPosition, double power) {
        while (Math.abs(targetPosition - fl.getCurrentPosition()) > 5) {
            if (targetPosition - fl.getCurrentPosition() > 5) {
                fl.setPower(power);
            }

            else if (targetPosition - fl.getCurrentPosition() < -5) {
                fl.setPower(-power);
            }
        }

        fl.setPower(0);
    }

    /*
    public void setDistance(int distance) {
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        while (motor.getCurrentPosition() < pos) {
            motor.setPower(0.5);
        }
        motor.setPower(0);
    }

     */

    public void setPower(double straight, double turn) {
        double LeftPower = straight - turn;
        double RightPower = straight + turn;
        setPower(LeftPower, RightPower, LeftPower, RightPower);
    }

    public void bangBangThing(double power, double angleTarget) {
        while (Math.abs((angleTarget - imu.getAngularOrientation().firstAngle)) > 0.05) {
            if (angleTarget - imu.getAngularOrientation().firstAngle > 0.05) {
                setPower(0,power);
            }

            else if (angleTarget - imu.getAngularOrientation().firstAngle < -0.05) {
                setPower(0,-power);
            }
        }
        
        setPower(0,0);

    }
}
