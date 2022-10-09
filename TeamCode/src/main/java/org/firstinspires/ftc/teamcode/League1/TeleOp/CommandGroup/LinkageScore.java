package org.firstinspires.ftc.teamcode.League1.TeleOp.CommandGroup;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.League1.Subsystems.ScoringSystem2;

public class LinkageScore extends SequentialCommandGroup {


    public LinkageScore(ScoringSystem2 score){
        super(
                new InstantCommand(() -> score.setLinkagePosition(0.7)),
                new WaitCommand(20),
                new InstantCommand(() -> score.setLinkagePosition(0.73)),
                new WaitCommand(20),
                new InstantCommand(() -> score.setLinkagePosition(0.76)),
                new WaitCommand(20),
                new InstantCommand(() -> score.setLinkagePosition(0.79)),
                new WaitCommand(20),
                new InstantCommand(() -> score.setLinkagePosition(0.82)),
                new WaitCommand(20),
                new InstantCommand(() -> score.setLinkagePosition(0.85)),
                new WaitCommand(20),
                new InstantCommand(() -> score.setLinkagePosition(0.88)),
                new WaitCommand(20),
                new InstantCommand(() -> score.setLinkagePosition(0.91)),
                new WaitCommand(20),
                new InstantCommand(() -> score.setLinkagePosition(0.95))




        );


    }
}
