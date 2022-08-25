package org.firstinspires.ftc.teamcode.Testing.Command.NewBot;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.Testing.RobotK;

public class LinkageUpSequence extends SequentialCommandGroup {



    public LinkageUpSequence(RobotK robot){
        super(

                //new InstantCommand(() -> robot.intake.brake()),
                //new InstantCommand(() -> robot.intake.clampAndRelease(true)),

                /*

            clampState = false;
            sleep(500);
            robot.lift.retract(0.5);
            sleep(500);
            robot.intake.setLinkage(0);

                 */


                new InstantCommand(() -> robot.intake.clampAndRelease(true)),
                new WaitCommand(500),
                new InstantCommand(() -> robot.intake.setLinkage(1))
        );


    }



}
