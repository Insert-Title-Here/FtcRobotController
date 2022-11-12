package org.firstinspires.ftc.teamcode.League1.TeleOp.CommandGroup;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.League1.Common.Constants;
import org.firstinspires.ftc.teamcode.League1.Subsystems.ScoringSystem2;
import org.firstinspires.ftc.teamcode.League1.TeleOp.ScoringSystemCommand;

public class LinkageScore extends SequentialCommandGroup {


    public LinkageScore(ScoringSystemCommand score, Constants constants){
        super(
                new InstantCommand(() -> score.setLinkagePositionLogistic(constants.linkageUp, 200)),
                new WaitCommand(200),

                new InstantCommand(() -> score.autoGoToPosition())


        );


    }
}
