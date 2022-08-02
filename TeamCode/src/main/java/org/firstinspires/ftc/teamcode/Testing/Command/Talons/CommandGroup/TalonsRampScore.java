package org.firstinspires.ftc.teamcode.Testing.Command.Talons.CommandGroup;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.Testing.Command.Talons.Command.RampCommand;
import org.firstinspires.ftc.teamcode.Testing.Command.Talons.TalonsScoringSystem;
import org.firstinspires.ftc.teamcode.Testing.Command.Talons.TalonsCommandTestingTeleOp;

public class TalonsRampScore extends SequentialCommandGroup {




    public TalonsRampScore(TalonsCommandTestingTeleOp.OpModeType type, TalonsScoringSystem lift){
        super(
                new InstantCommand(() -> lift.houseSetPosition(0.5)),
                new RampCommand(lift),


                new InstantCommand(() -> lift.open()),
                new InstantCommand(() -> lift.down())






        );


    }



}
