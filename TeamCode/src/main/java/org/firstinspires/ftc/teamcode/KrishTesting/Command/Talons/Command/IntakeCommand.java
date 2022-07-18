package org.firstinspires.ftc.teamcode.KrishTesting.Command.Talons.Command;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.KrishTesting.Command.Talons.RobotT;

public class IntakeCommand extends CommandBase {

    RobotT robot;
    boolean intaken;
    Gamepad gamepad;

    public IntakeCommand(RobotT robot, Gamepad gamepad){
        this.robot = robot;
        this.gamepad = gamepad;
    }

    @Override
    public void execute() {
        robot.intake.setPower(1);

        if(robot.color.getDistance(DistanceUnit.INCH) < 1 || gamepad.start){
            intaken = true;
            robot.intake.brake();
        }




    }

    @Override
    public boolean isFinished() {
        if(intaken){
            return true;
        }else{
            return false;
        }
    }
}
