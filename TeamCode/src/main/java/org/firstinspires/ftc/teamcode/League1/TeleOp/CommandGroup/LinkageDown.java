package org.firstinspires.ftc.teamcode.League1.TeleOp.CommandGroup;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.League1.Common.Constants;
import org.firstinspires.ftc.teamcode.League1.Common.Robot;
import org.firstinspires.ftc.teamcode.League1.Subsystems.ScoringSystem2;
import org.firstinspires.ftc.teamcode.League1.TeleOp.Command.GrabCommand;
import org.firstinspires.ftc.teamcode.League1.TeleOp.Command.LinkageCommand;

public class LinkageDown extends SequentialCommandGroup {


    public LinkageDown(ScoringSystem2 score){

        super(
                new InstantCommand(() -> score.setLinkagePosition(0.7)),
                new WaitCommand(20),
                new InstantCommand(() -> score.setLinkagePosition(0.67)),
                new WaitCommand(20),
                new InstantCommand(() -> score.setLinkagePosition(0.64)),
                new WaitCommand(20),
                new InstantCommand(() -> score.setLinkagePosition(0.61)),
                new WaitCommand(20),
                new InstantCommand(() -> score.setLinkagePosition(0.58)),
                new WaitCommand(20),
                new InstantCommand(() -> score.setLinkagePosition(0.55)),
                new WaitCommand(20),
                new InstantCommand(() -> score.setLinkagePosition(0.52)),
                new WaitCommand(20),
                new InstantCommand(() -> score.setLinkagePosition(0.49)),
                new WaitCommand(20),
                new InstantCommand(() -> score.setLinkagePosition(0.46)),
                new WaitCommand(20),
                new InstantCommand(() -> score.setLinkagePosition(0.43)),
                new WaitCommand(20),
                new InstantCommand(() -> score.setLinkagePosition(0.4)),
                new WaitCommand(20),
                new InstantCommand(() -> score.setLinkagePosition(0.37)),
                new WaitCommand(20),
                new InstantCommand(() -> score.setLinkagePosition(0.34)),
                new WaitCommand(20),
                new InstantCommand(() -> score.setLinkagePosition(0.31)),
                new WaitCommand(20),
                new InstantCommand(() -> score.setLinkagePosition(0.28)),
                new WaitCommand(20),
                new InstantCommand(() -> score.setLinkagePosition(0.25)),
                new WaitCommand(20),
                new InstantCommand(() -> score.setLinkagePosition(0.22)),
                new WaitCommand(20),
                new InstantCommand(() -> score.setLinkagePosition(0.19)),
                new WaitCommand(20),
                new InstantCommand(() -> score.setLinkagePosition(0.16)),
                new WaitCommand(20),
                new InstantCommand(() -> score.setLinkagePosition(0.13)),
                new WaitCommand(20),
                new InstantCommand(() -> score.setLinkagePosition(0.1)),
                new WaitCommand(20),
                new InstantCommand(() -> score.setLinkagePosition(0.07)),
                new WaitCommand(20),
                new InstantCommand(() -> score.setLinkagePosition(0.03))

        );
    }
}
