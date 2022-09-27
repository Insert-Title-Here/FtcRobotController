package org.firstinspires.ftc.teamcode.League1.TeleOp.CommandGroup;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.League1.Common.Constants;
import org.firstinspires.ftc.teamcode.League1.Common.Robot;
import org.firstinspires.ftc.teamcode.League1.Subsystems.ScoringSystem;
import org.firstinspires.ftc.teamcode.League1.TeleOp.Command.GrabCommand;
import org.firstinspires.ftc.teamcode.League1.TeleOp.Command.LinkageCommand;

public class LinkageDown extends SequentialCommandGroup {


    public LinkageDown(ScoringSystem score){
        super(
            new InstantCommand(() -> score.setLinkagePosition(0.4)),
            new WaitCommand(500),
            new InstantCommand(() -> score.setLinkagePosition(0.3)),
            new WaitCommand(500),
            new InstantCommand(() -> score.setLinkagePosition(0.2)),
            new WaitCommand(500),
            new InstantCommand(() -> score.setLinkagePosition(0.1)),
            new WaitCommand(500),
            new InstantCommand(() -> score.setLinkagePosition(0.05))
        );
    }
}
