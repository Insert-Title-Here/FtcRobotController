package org.firstinspires.ftc.teamcode.KrishTesting.Command.Talons.CommandGroup;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.Intake;
import org.firstinspires.ftc.teamcode.KrishTesting.Command.Talons.Command.IntakeCommand;
import org.firstinspires.ftc.teamcode.KrishTesting.Command.Talons.Command.RampCommand;
import org.firstinspires.ftc.teamcode.KrishTesting.Command.Talons.RobotT;
import org.firstinspires.ftc.teamcode.KrishTesting.Command.Talons.TalonsCommandTestingTeleOp;
import org.firstinspires.ftc.teamcode.KrishTesting.RobotK;

public class TalonsLiftingSequence extends SequentialCommandGroup {




    public TalonsLiftingSequence(TalonsCommandTestingTeleOp.OpModeType type, RobotT robot, Gamepad gamepad){
        super(
                new IntakeCommand(robot, gamepad),

                new InstantCommand(() -> robot.intake.brake()),
                new InstantCommand(() -> robot.lift.close()),

                new WaitCommand(100),
                new InstantCommand(() -> robot.lift.linkageSetPosition(0.65))
                /*new WaitCommand(300),
                new InstantCommand(() -> robot.lift.houseSetPosition(0.5)),
                new RampCommand(robot),


                new InstantCommand(() -> robot.lift.open()),
                new InstantCommand(() -> robot.lift.down())

                 */




        );


    }



}
