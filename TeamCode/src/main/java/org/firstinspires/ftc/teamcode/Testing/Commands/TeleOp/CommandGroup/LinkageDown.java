package org.firstinspires.ftc.teamcode.Testing.Commands.TeleOp.CommandGroup;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.Competition.Interleagues.Common.Constants;
import org.firstinspires.ftc.teamcode.Testing.Commands.Subsystems.ScoringSystemCommand;

public class LinkageDown extends SequentialCommandGroup {


    public LinkageDown(ScoringSystemCommand score /*Constants constants*/){

        super(
                new InstantCommand(() -> score.setLinkagePositionLogistic(Constants.linkageDown, 300))

        );
    }
}
