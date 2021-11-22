package org.firstinspires.ftc.teamcode.MecanumCode.Common;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class MagneticArm {

    public enum magnetState {
        OPEN,
        GRABBING
    }

    DcMotor magneticExtension, magneticExtensionEncoder;
    CRServo magneticExtensionSM;

    Servo magnet;
    Servo level;

    double levelPosition;

    public MagneticArm(HardwareMap hardwareMap) {
        //magneticExtension = hardwareMap.get(DcMotor.class, "MagneticArm");

        magnet = hardwareMap.get(Servo.class, "Magnet");
        level = hardwareMap.get(Servo.class, "Level");
        //magneticExtension.setDirection(DcMotor.Direction.FORWARD);
        //magneticExtension.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        magneticExtensionEncoder = hardwareMap.dcMotor.get("FrontRightDrive");
        magneticExtensionSM = hardwareMap.crservo.get("MagExtension");
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


    /**
     * same code using a sparkMini
     * @param armPosition
     */
    public void setArmPositionSM(int armPosition){
        magneticExtensionEncoder.setTargetPosition(armPosition);

        magneticExtensionEncoder.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        while (Math.abs(magneticExtensionEncoder.getCurrentPosition() - magneticExtensionEncoder.getTargetPosition()) > 10) {
            magneticExtensionSM.setPower(0.5);
        }

        magneticExtensionSM.setPower(0);

        magneticExtensionEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }

    public void setExtensionSMPower(double power) {
        magneticExtensionSM.setPower(power);
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
