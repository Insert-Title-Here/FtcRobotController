package org.firstinspires.ftc.teamcode.League1.TeleOp.CommandGroup;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.League1.Common.Constants;
import org.firstinspires.ftc.teamcode.League1.Common.Robot;
import org.firstinspires.ftc.teamcode.League1.Subsystems.ScoringSystem;
import org.firstinspires.ftc.teamcode.League1.Subsystems.ScoringSystem2;
import org.firstinspires.ftc.teamcode.League1.TeleOp.Command.GrabCommand;
import org.firstinspires.ftc.teamcode.League1.TeleOp.Command.LinkageCommand;

public class GrabAndScore extends SequentialCommandGroup {


    public GrabAndScore(ScoringSystem2 score, Constants constants){
        super(

                new InstantCommand(() -> score.setGrabberPosition(constants.grabbing)),

                new WaitCommand(200),

                new ParallelCommandGroup(
                        new InstantCommand(() -> score.autoGoToPosition()),

                        //TODO: Test how this works
                        new LinkageScore(score)
                )


        );

        score.setGrabbing(true);
        score.setLinkageUp(true);
    }
}
