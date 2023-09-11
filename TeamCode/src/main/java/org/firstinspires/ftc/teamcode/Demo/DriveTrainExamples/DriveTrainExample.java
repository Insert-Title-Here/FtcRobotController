package org.firstinspires.ftc.teamcode.Demo.DriveTrainExamples;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class DriveTrainExample {

    public enum MovementType{
        STRAIGHT,
        STRAFE,
        ROTATE
    }

    DcMotor fl, fr, bl, br;

    public DriveTrainExample(HardwareMap hardwareMap) {
        // Change deviceName to whatever motors are called in config
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

        // might need to change this so that robot drives forwards when all powers are positive
        fl.setDirection(DcMotor.Direction.FORWARD);
        fr.setDirection(DcMotor.Direction.REVERSE);
        bl.setDirection(DcMotor.Direction.FORWARD);
        br.setDirection(DcMotor.Direction.REVERSE);

    }

    public void setPower(double flPow, double frPow, double blPow, double brPow) {
        fl.setPower(-flPow);
        fr.setPower(frPow) ;
        bl.setPower(-blPow);
        br.setPower(brPow);
    }

    public void setPowerAuto(double power, MovementType movement) {
        if(movement == MovementType.STRAIGHT) {
            setPower(power, power, power, power);
        }else if(movement == MovementType.STRAFE){
            setPower(power, -power, -power, power);
        }else if(movement == MovementType.ROTATE){
            setPower(power, -power, power, -power);
        }
    }

}
