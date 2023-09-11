package org.firstinspires.ftc.teamcode.Competition.State.Autonomous.InUse.CustomCommands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;

public class TimeFailSafe extends CommandBase {

    ElapsedTime time;
    double startTime;


    public TimeFailSafe(ElapsedTime time, double startTime){
        this.time = time;
        this.startTime = startTime;
    }

    @Override
    public void initialize() {

        if(time.seconds() - startTime > 25){

        }

    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
