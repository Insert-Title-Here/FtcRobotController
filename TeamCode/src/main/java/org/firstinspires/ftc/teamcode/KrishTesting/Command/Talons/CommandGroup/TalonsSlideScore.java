package org.firstinspires.ftc.teamcode.KrishTesting.Command.Talons.CommandGroup;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.KrishTesting.Command.Talons.Command.RampCommand;
import org.firstinspires.ftc.teamcode.KrishTesting.Command.Talons.RobotT;
import org.firstinspires.ftc.teamcode.KrishTesting.Command.Talons.TalonsCommandTestingTeleOp;

public class TalonsSlideScore extends SequentialCommandGroup {




    public TalonsSlideScore(TalonsCommandTestingTeleOp.OpModeType type, RobotT robot){
        super(
                new InstantCommand(() -> robot.lift.extend(0.5)),


                new InstantCommand(() -> robot.lift.score())

                //new InstantCommand(() -> robot.lift.retract(0.5)),
                //new InstantCommand(() -> robot.lift.open()),

                //new InstantCommand(() -> robot.lift.down())








        );


    }



}
