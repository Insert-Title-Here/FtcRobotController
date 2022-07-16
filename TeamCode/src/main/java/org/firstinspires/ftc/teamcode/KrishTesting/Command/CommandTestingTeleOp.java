package org.firstinspires.ftc.teamcode.KrishTesting.Command;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.Robot;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.gamepad.KeyReader;

import org.firstinspires.ftc.teamcode.KrishTesting.RobotK;

import java.io.FileNotFoundException;
import java.util.function.BooleanSupplier;

public class CommandTestingTeleOp extends CommandOpMode {

    private RobotK robot;

    @Override
    public void initialize() {
        try {
            robot = new RobotK(hardwareMap);
        } catch (FileNotFoundException e) {
            e.printStackTrace();
        }

    }

    @Override
    public void run() {
        super.run();
        robot.update();


    }
}
