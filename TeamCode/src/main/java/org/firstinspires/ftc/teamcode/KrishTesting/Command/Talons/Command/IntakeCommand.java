package org.firstinspires.ftc.teamcode.KrishTesting.Command.Talons.Command;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.KrishTesting.Command.Talons.RobotT;
import org.firstinspires.ftc.teamcode.KrishTesting.Command.Talons.TalonsIntake;

public class IntakeCommand extends CommandBase {

    TalonsIntake intake;
    RobotT robot;
    boolean intaken;
    Gamepad gamepad;

    public IntakeCommand(TalonsIntake intake, Gamepad gamepad, RobotT robot){
        this.intake = intake;
        this.gamepad = gamepad;
        this.robot = robot;
    }

    @Override
    public void execute() {
        intake.setPower(1);

        if(robot.color.getDistance(DistanceUnit.INCH) < 1 || gamepad.start){
            intaken = true;
            intake.brake();
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
