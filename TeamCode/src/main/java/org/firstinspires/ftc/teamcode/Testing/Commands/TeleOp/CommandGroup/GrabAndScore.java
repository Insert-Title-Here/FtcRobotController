package org.firstinspires.ftc.teamcode.Testing.Commands.TeleOp.CommandGroup;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.Competition.Interleagues.Common.Constants;
import org.firstinspires.ftc.teamcode.Testing.Commands.Subsystems.ScoringSystemCommand;

public class GrabAndScore extends SequentialCommandGroup {


    public GrabAndScore(ScoringSystemCommand score/*Constants constants*/){
        super(

                new InstantCommand(() -> score.setGrabberPosition(Constants.grabbing)),

                new WaitCommand(200),


                new LinkageScore(score),




                new InstantCommand(() -> score.setLinkagePositionLogistic(Constants.score, 200))


                //TODO: Test how this works






        );

        score.setGrabbing(true);
        score.setLinkageUp(true);
    }
}
