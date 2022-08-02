package org.firstinspires.ftc.teamcode.Testing.Command.Talons.Command;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Testing.Command.Talons.TalonsScoringSystem;

public class RampCommand extends CommandBase {

    TalonsScoringSystem lift;
    ElapsedTime timer;

    public RampCommand(TalonsScoringSystem lift){
        this.lift = lift;
        timer = new ElapsedTime();
    }

    @Override
    public void execute() {
        lift.setRampPower(1);







    }

    @Override
    public boolean isFinished() {
        if(timer.seconds() > 1.5){
            lift.setRampPower(0);
            return true;
        }else{
            return false;
        }
    }
}
