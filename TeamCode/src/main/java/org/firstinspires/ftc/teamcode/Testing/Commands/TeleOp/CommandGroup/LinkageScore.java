package org.firstinspires.ftc.teamcode.Testing.Commands.TeleOp.CommandGroup;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.Competition.Interleagues.Common.Constants;
import org.firstinspires.ftc.teamcode.Testing.Commands.Subsystems.ScoringSystemCommand;

public class LinkageScore extends SequentialCommandGroup {


    public LinkageScore(ScoringSystemCommand score /*Constants constants*/){
        super(
                new InstantCommand(() -> score.setLinkagePositionLogistic(Constants.linkageUp, 200)),
                new WaitCommand(200),

                new InstantCommand(() -> score.autoGoToPosition())


        );


    }
}
