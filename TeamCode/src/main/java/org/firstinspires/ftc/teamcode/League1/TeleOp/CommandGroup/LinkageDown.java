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
import org.firstinspires.ftc.teamcode.League1.TeleOp.ScoringSystemCommand;

public class LinkageDown extends SequentialCommandGroup {


    public LinkageDown(ScoringSystemCommand score /*Constants constants*/){

        super(
                new InstantCommand(() -> score.setLinkagePositionLogistic(Constants.linkageDown, 300))

        );
    }
}
