package org.firstinspires.ftc.teamcode.Testing.Command.NewBot;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.Testing.RobotK;

public class ScoreSequence extends SequentialCommandGroup {

    public ScoreSequence(RobotK robot){
        super(



                new InstantCommand(() -> robot.intake.clampAndRelease(false)),
                new WaitCommand(1000),
                new InstantCommand(() -> robot.intake.setLinkage(0)),
                new WaitCommand(500),
                new InstantCommand(() -> robot.lift.retract(0.3))

        );


    }



}
