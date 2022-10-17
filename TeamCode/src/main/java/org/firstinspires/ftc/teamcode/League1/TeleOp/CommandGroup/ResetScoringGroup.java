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
import org.firstinspires.ftc.teamcode.League1.TeleOp.Command.LiftCommand;
import org.firstinspires.ftc.teamcode.League1.TeleOp.Command.LinkageCommand;

public class ResetScoringGroup extends SequentialCommandGroup {


    public ResetScoringGroup(ScoringSystem2 score, Constants constants){
        super(

                new InstantCommand(() -> score.setGrabberPosition(constants.open)),

                new WaitCommand(200),

                new ParallelCommandGroup(
                        new InstantCommand(() -> score.moveToPosition(0, 0.5)),

                        //TODO: Test how this works
                        new LinkageDown(score)
                )



        );

        score.setGrabbing(false);
        score.setLinkageUp(false);

    }
}
