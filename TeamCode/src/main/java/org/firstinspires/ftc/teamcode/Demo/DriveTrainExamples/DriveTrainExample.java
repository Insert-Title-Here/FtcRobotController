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

    // constructor - initializes and sets up motors
    public DriveTrainExample(HardwareMap hardwareMap) {
        // Change deviceName to whatever motors are called in config
        fl = hardwareMap.get(DcMotor.class, "FrontLeftDrive");
        fr = hardwareMap.get(DcMotor.class, "FrontRightDrive");
        bl = hardwareMap.get(DcMotor.class, "BackLeftDrive");
        br = hardwareMap.get(DcMotor.class, "BackRightDrive");

        // makes sure all encoders are set to 0
        fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        br.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // runs motors without a PID controller - does NOT mean you can't use the encoder
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

    // set power to motors individually based on passed in parameters
    public void setPower(double flPow, double frPow, double blPow, double brPow) {
        fl.setPower(-flPow);
        fr.setPower(frPow) ;
        bl.setPower(-blPow);
        br.setPower(brPow);
    }

    // basic mecanum stuff - ignore for now
    public void setPowerAuto(double power, MovementType movement) {
        if(movement == MovementType.STRAIGHT) {
            setPower(power, power, power, power);
        }else if(movement == MovementType.STRAFE){
            setPower(power, -power, -power, power);
        }else if(movement == MovementType.ROTATE){
            setPower(power, -power, power, -power);
        }
    }

    // sets power based on a straight and a turn value, allowing
    // for smooth driving with two sticks
    public void setPower(double straight, double turn) {
        double leftPower = straight - turn;
        double rightPower = straight + turn;
        fl.setPower(leftPower);
        fr.setPower(rightPower);
        bl.setPower(leftPower);
        br.setPower(rightPower);
    }

}
