package org.firstinspires.ftc.teamcode.Testing.Commands.TeleOp.CommandGroup;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.Competition.Interleagues.Common.Constants;
import org.firstinspires.ftc.teamcode.Testing.Commands.Subsystems.ScoringSystemCommand;

public class ResetScoringGroup extends SequentialCommandGroup {


    public ResetScoringGroup(ScoringSystemCommand score/*Constants constants*/){
        super(

                new InstantCommand(() -> score.setGrabberPosition(Constants.open)),

                new WaitCommand(200),

                new ParallelCommandGroup(
                        new InstantCommand(() -> score.moveToPosition(0, 0.5)),

                        //TODO: Test how this works
                        new LinkageDown(score)
                )



        );

        score.setGrabbing(false);
        score.setLinkageUp(false);
        score.setExtended(false);

    }
}
