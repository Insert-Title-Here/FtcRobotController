package org.firstinspires.ftc.teamcode.KrishTesting.Command.Talons.CommandGroup;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.KrishTesting.Command.Talons.Command.IntakeCommand;
import org.firstinspires.ftc.teamcode.KrishTesting.Command.Talons.Command.RampCommand;
import org.firstinspires.ftc.teamcode.KrishTesting.Command.Talons.RobotT;
import org.firstinspires.ftc.teamcode.KrishTesting.Command.Talons.TalonsCommandTestingTeleOp;

public class TalonsRampScore extends SequentialCommandGroup {




    public TalonsRampScore(TalonsCommandTestingTeleOp.OpModeType type, RobotT robot){
        super(
                new InstantCommand(() -> robot.lift.houseSetPosition(0.5)),
                new RampCommand(robot),


                new InstantCommand(() -> robot.lift.open()),
                new InstantCommand(() -> robot.lift.down())






        );


    }



}
