package org.firstinspires.ftc.teamcode.KrishTesting.Command.Talons.Command;

import static org.firstinspires.ftc.teamcode.Utils.sleep;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.KrishTesting.Command.Talons.RobotT;

public class RampCommand extends CommandBase {

    RobotT robot;
    ElapsedTime timer;

    public RampCommand(RobotT robot){
        this.robot = robot;
        timer = new ElapsedTime();
    }

    @Override
    public void execute() {
        robot.lift.setRampPower(1);







    }

    @Override
    public boolean isFinished() {
        if(timer.seconds() > 2){
            return true;
        }else{
            return false;
        }
    }
}
