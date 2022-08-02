package org.firstinspires.ftc.teamcode.Testing.Command.Talons.CommandGroup;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.Testing.Command.Talons.Command.IntakeCommand;
import org.firstinspires.ftc.teamcode.Testing.Command.Talons.RobotT;
import org.firstinspires.ftc.teamcode.Testing.Command.Talons.TalonsScoringSystem;
import org.firstinspires.ftc.teamcode.Testing.Command.Talons.TalonsCommandTestingTeleOp;
import org.firstinspires.ftc.teamcode.Testing.Command.Talons.TalonsIntake;

public class TalonsLiftingSequence extends SequentialCommandGroup {




    public TalonsLiftingSequence(TalonsCommandTestingTeleOp.OpModeType type, TalonsIntake intake, TalonsScoringSystem lift, Gamepad gamepad, RobotT robot){
        super(
                new IntakeCommand(intake, gamepad, robot),

                new InstantCommand(() -> intake.brake()),
                new InstantCommand(() -> lift.close()),

                new WaitCommand(100),
                new InstantCommand(() -> lift.linkageSetPosition(0.65))
                /*new WaitCommand(300),
                new InstantCommand(() -> robot.lift.houseSetPosition(0.5)),
                new RampCommand(robot),


                new InstantCommand(() -> robot.lift.open()),
                new InstantCommand(() -> robot.lift.down())

                 */




        );


    }



}
