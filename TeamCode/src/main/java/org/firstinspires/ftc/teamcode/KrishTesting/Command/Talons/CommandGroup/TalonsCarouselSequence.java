package org.firstinspires.ftc.teamcode.KrishTesting.Command.Talons.CommandGroup;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.KrishTesting.Command.Talons.Command.CarouselCommand;
import org.firstinspires.ftc.teamcode.KrishTesting.Command.Talons.Command.IntakeCommand;
import org.firstinspires.ftc.teamcode.KrishTesting.Command.Talons.RobotT;
import org.firstinspires.ftc.teamcode.KrishTesting.Command.Talons.TalonsCommandTestingTeleOp;
import org.firstinspires.ftc.teamcode.KrishTesting.Command.Talons.TalonsEndGame;
import org.firstinspires.ftc.teamcode.KrishTesting.Command.Talons.TalonsIntake;
import org.firstinspires.ftc.teamcode.KrishTesting.Command.Talons.TalonsScoringSystem;

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
