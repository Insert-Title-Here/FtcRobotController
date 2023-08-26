package org.firstinspires.ftc.teamcode.Demo.DriveTrainExamples;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class DriveTrainYay {

    DcMotor fl;
    DcMotor fr;
    DcMotor bl;
    DcMotor br;
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

    public void setPower(double straight, double turn) {
        double LeftPower = straight - turn;
        double RightPower = straight + turn;
        setPower(LeftPower, RightPower, LeftPower, RightPower);
    }
}
