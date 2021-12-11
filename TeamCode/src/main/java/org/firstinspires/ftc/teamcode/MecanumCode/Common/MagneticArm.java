package org.firstinspires.ftc.teamcode.MecanumCode.Common;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
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
    double extensionPower;

    public MagneticArm(HardwareMap hardwareMap) {
        magneticExtension = hardwareMap.get(DcMotor.class, "MagneticArm");

        magnet = hardwareMap.get(Servo.class, "Magnet");
        level = hardwareMap.get(Servo.class, "Level");
        magneticExtension.setDirection(DcMotor.Direction.FORWARD);
        magneticExtension.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //magneticExtensionEncoder = hardwareMap.dcMotor.get("FrontLeftDrive");
        magneticExtension.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        magneticExtension.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //magneticExtensionSM = hardwareMap.crservo.get("MagExtension");
        magnet.setPosition(0.97);

        levelPosition = 0.9;
        level.setPosition(levelPosition);
        //magneticExtensionSM.setPower(0);
    }

    public void increaseLevelPosition(double increment) {
        if (levelPosition + increment < 0.9) {
            levelPosition += increment;
        } else {
            levelPosition = 1;
        }
        level.setPosition(levelPosition);
    }

    public void decreaseLevelPosition(double increment) {
        if (levelPosition - increment > 0.1) {
            levelPosition -= increment;
        } else {
            levelPosition = 0.29;
        }
        level.setPosition(levelPosition);
    }


    /**
     * same code using a sparkMini
     * @param armPosition
     */
    public void setArmPositionSM(int armPosition, LinearOpMode currentOpMode){
        magneticExtensionEncoder.setTargetPosition(armPosition);

        magneticExtensionEncoder.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        extensionPower = 0.5;

        while (Math.abs(magneticExtensionEncoder.getCurrentPosition() - magneticExtensionEncoder.getTargetPosition()) > 10 &&
        currentOpMode.opModeIsActive()) {
            magneticExtensionSM.setPower(-0.5);

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
            magnet.setPosition(0.97);
        } else if(position == magnetState.OPEN) {
            magnet.setPosition(0.5);
        }
    }

    public void setArmPosition(int armPosition) {
        magneticExtension.setTargetPosition(armPosition);

        magneticExtension.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        magneticExtension.setPower(0.75);

        while (magneticExtension.isBusy()) {

        }

        magneticExtension.setPower(0);

        magneticExtension.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }

    // 0.55 fully retracted, 0.45 fully extended
    public double getLevelPosition() {
        if(magneticExtension.getCurrentPosition() < 0 && magneticExtension.getCurrentPosition() > -350 ) {
            return (magneticExtension.getCurrentPosition() / 3500.0) + 0.55;
        } else {
            return 0.45;
        }
    }

    public void setExtensionPower(double power) {
        magneticExtension.setPower(power);
    }

    public double[] getTelemetry() {
        return new double[]{levelPosition, level.getPosition(), magnet.getPosition()};
    }

    public int getEncoderTics() {
        return magneticExtension.getCurrentPosition();
    }


}
