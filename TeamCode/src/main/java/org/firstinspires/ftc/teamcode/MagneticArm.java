package org.firstinspires.ftc.teamcode;

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

    //This is how I had it set up in my teleop while I was testing. You can do whatever you want with it.
    /*
    if(gamepad1.dpad_up) {
        arm.increaseLevelPosition(0.1);
        while(gamepad1.dpad_up) {
        }
    }

    if(gamepad1.dpad_down) {
        arm.decreaseLevelPosition(0.1);
        while(gamepad1.dpad_down) {

        }
    }

    if(gamepad1.a) {
        // Fully extend arm
        arm.setArmPosition(-300);
        while(gamepad1.a) {

        }
    }

    if(gamepad1.b) {
        // Lower level to cube height
        arm.setLevelPosition(0.5);
    }

    if(gamepad1.y) {
        // Raise level
        arm.setLevelPosition(1);
    }

    if(gamepad1.x) {
        // Drop cube and retract arm
        arm.setMagnetPosition(MagneticArm.magnetState.OPEN);
        sleep(1000);
        arm.setMagnetPosition(MagneticArm.magnetState.GRABBING);
        arm.setArmPosition(0);
    }

    if (gamepad1.left_trigger > 0.1) {
        arm.magneticExtension.setPower(gamepad1.left_trigger);
    } else if (gamepad1.right_trigger > 0.1) {
        arm.magneticExtension.setPower(-gamepad1.right_trigger);
    } else {
        arm.magneticExtension.setPower(0);
    }

     */

}
