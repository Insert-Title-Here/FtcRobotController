package org.firstinspires.ftc.teamcode.League1.TeleOp.CommandGroup;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.League1.Subsystems.ScoringSystem;

public class LinkageUp extends SequentialCommandGroup {


    public LinkageUp(ScoringSystem score){
        super(
            new InstantCommand(() -> score.setLinkagePosition(0.6)),
            new WaitCommand(500),
            new InstantCommand(() -> score.setLinkagePosition(0.7)),
            new WaitCommand(500),
            new InstantCommand(() -> score.setLinkagePosition(0.8)),
            new WaitCommand(500),
            new InstantCommand(() -> score.setLinkagePosition(0.9)),
            new WaitCommand(500),
            new InstantCommand(() -> score.setLinkagePosition(1))
        );
    }
}
