package org.firstinspires.ftc.teamcode.KrishTesting.Command;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Intake;

/**
 * A gripper mechanism that grabs a stone from the quarry.
 * Centered around the Skystone game for FTC that was done in the 2019
 * to 2020 season.
 */
public class TestingSubsystem extends SubsystemBase {

    private Servo lJoint, rJoint;
    private CRServo lIntake, rIntake;


    public TestingSubsystem(HardwareMap hardwareMap) {
        lJoint = hardwareMap.get(Servo.class, "lJoint");
        rJoint = hardwareMap.get(Servo.class, "rJoint");
        lIntake = hardwareMap.get(CRServo.class, "lIntake");
        rIntake = hardwareMap.get(CRServo.class, "rIntake");

        lJoint.setPosition(1);
        rJoint.setPosition(0);
    }

    /**
     * Grabs a stone.
     */
    public void clamp() {
        lJoint.setPosition(0.9);
        rJoint.setPosition(0.1);
    }

    /**
     * Releases a stone.
     */
    public void release() {
        lJoint.setPosition(1);
        rJoint.setPosition(0);
    }

}