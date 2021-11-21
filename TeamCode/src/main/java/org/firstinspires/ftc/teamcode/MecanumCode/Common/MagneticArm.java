package org.firstinspires.ftc.teamcode.MecanumCode.Common;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class MagneticArm {

    public enum magnetState {
        OPEN,
        GRABBING
    }

    DcMotor magneticExtension;
    Servo magnet;
    Servo level;

    double levelPosition;

    public MagneticArm(HardwareMap hardwareMap) {
        magneticExtension = hardwareMap.get(DcMotor.class, "MagneticArm");
        magnet = hardwareMap.get(Servo.class, "Magnet");
        level = hardwareMap.get(Servo.class, "Level");
        magneticExtension.setDirection(DcMotor.Direction.FORWARD);
        magneticExtension.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        magnet.setPosition(1);
        level.setPosition(levelPosition);
    }

    public void increaseLevelPosition(double increment) {
        if (levelPosition + increment < 1) {
            levelPosition += increment;
        } else {
            levelPosition = 1;
        }
        level.setPosition(levelPosition);
    }

    public void decreaseLevelPosition(double increment) {
        if (levelPosition - increment > 0) {
            levelPosition -= increment;
        } else {
            levelPosition = 0;
        }
        level.setPosition(levelPosition);
    }

    public void setLevelPosition(double position) {
        levelPosition = position;
        level.setPosition(levelPosition);
    }

    public void setMagnetPosition(magnetState position) {
        if(position == magnetState.GRABBING) {
            magnet.setPosition(1);
        } else if(position == magnetState.OPEN) {
            magnet.setPosition(0.5);
        }
    }

    public void setArmPosition(int armPosition) {
        magneticExtension.setTargetPosition(armPosition);

        magneticExtension.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        magneticExtension.setPower(0.5);

        while (magneticExtension.isBusy()) {

        }

        magneticExtension.setPower(0);

        magneticExtension.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }

    public void setExtensionPower(double power) {
        magneticExtension.setPower(power);
    }

}
