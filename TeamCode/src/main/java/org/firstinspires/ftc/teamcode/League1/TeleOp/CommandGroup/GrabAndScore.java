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
import org.firstinspires.ftc.teamcode.League1.TeleOp.ScoringSystemCommand;

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
