package org.firstinspires.ftc.teamcode.Testing.Command.Talons.CommandGroup;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.Testing.Command.Talons.TalonsCommandTestingTeleOp;
import org.firstinspires.ftc.teamcode.Testing.Command.Talons.TalonsScoringSystem;

public class TalonsSlideScore extends SequentialCommandGroup {




    public TalonsSlideScore(TalonsCommandTestingTeleOp.OpModeType type, TalonsScoringSystem lift, Gamepad gamepad){
        super(

                new InstantCommand(() -> lift.extend(1, gamepad)),
                new InstantCommand(() -> lift.score()),

                //new InstantCommand(() -> robot.lift.retract(0.5)),
                new WaitCommand(600),
                new InstantCommand(() -> lift.setPower(0)),
                new InstantCommand(() -> lift.open()),
                new WaitCommand(500),
                new InstantCommand(() -> lift.down())








        );


    }



}
