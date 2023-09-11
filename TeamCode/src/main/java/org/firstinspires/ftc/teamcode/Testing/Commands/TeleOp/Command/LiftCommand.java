package org.firstinspires.ftc.teamcode.Testing.Commands.TeleOp.Command;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Competition.Interleagues.Common.Robot;
import org.firstinspires.ftc.teamcode.Competition.Interleagues.Subsystems.Older.ScoringSystem;

public class LiftCommand extends CommandBase {
    ScoringSystem score;
    private ScoringSystem.ExtensionHeight position;
    private double power;




    public LiftCommand(ScoringSystem score, /*//Constants constants,*/ HardwareMap hardwareMap, Robot robot, ScoringSystem.ExtensionHeight height, double power){
        score = new ScoringSystem(hardwareMap, robot,  false);
        this.position = position;
        this.power = power;



    }

    @Override
    public void initialize() {

        //TODO: change this later so that it accepts enum
        //score.moveToPosition(position, power);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
