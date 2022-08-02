package org.firstinspires.ftc.teamcode.Testing.Command.Talons.CommandGroup;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.Testing.Command.Talons.Command.CarouselCommand;
import org.firstinspires.ftc.teamcode.Testing.Command.Talons.TalonsCommandTestingTeleOp;
import org.firstinspires.ftc.teamcode.Testing.Command.Talons.TalonsEndGame;

public class TalonsCarouselSequence extends SequentialCommandGroup {




    public TalonsCarouselSequence(TalonsCommandTestingTeleOp.OpModeType type, TalonsEndGame endgame, Gamepad gamepad){
        super(

                new CarouselCommand(endgame, gamepad)

                /*new WaitCommand(300),
                new InstantCommand(() -> robot.lift.houseSetPosition(0.5)),
                new RampCommand(robot),


                new InstantCommand(() -> robot.lift.open()),
                new InstantCommand(() -> robot.lift.down())

                 */




        );


    }



}
