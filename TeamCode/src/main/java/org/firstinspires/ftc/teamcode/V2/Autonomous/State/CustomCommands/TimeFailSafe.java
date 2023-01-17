package org.firstinspires.ftc.teamcode.V2.Autonomous.State.CustomCommands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.V2.NewSubsystem.MecDriveV2;

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
