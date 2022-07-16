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
    public Intake.ConstantState constantState;
    private Intake.ConstantState previousState;

    public TestingSubsystem(HardwareMap hMap, String name) {
        //mechRotation = hMap.get(Servo.class, name);
    }

    /**
     * Grabs a stone.
     */
    public void grab() {
        //mechRotation.setPosition(0.76);
    }

    /**
     * Releases a stone.
     */
    public void release() {
        //mechRotation.setPosition(0);
    }

}