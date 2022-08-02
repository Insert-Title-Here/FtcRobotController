package org.firstinspires.ftc.teamcode.Testing.Command.NewBot;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.Testing.RobotK;

public class ScoreSequence extends CommandBase {


    public enum OpModeType {
        TELEOP, AUTO
    }

    public ScoreSequence(OpModeType type, RobotK robot){
        new SequentialCommandGroup(
                //new InstantCommand(() -> robot.intake.brake()),
                //new InstantCommand(() -> robot.intake.clampAndRelease(true)),

                //TODO: need to add linkage part ("house")

                //TODO: Need to tune position and power
                new InstantCommand(() -> robot.lift.extend(0.5)),

                new ParallelCommandGroup(
                        //new InstantCommand(() -> robot.intake.clampAndRelease(false)),
                        //new InstantCommand(() -> robot.intake.setPower(false, 0.5))
                )
        );


    }



}
