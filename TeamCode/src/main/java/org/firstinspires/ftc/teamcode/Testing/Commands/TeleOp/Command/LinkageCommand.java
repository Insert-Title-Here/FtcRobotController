package org.firstinspires.ftc.teamcode.Testing.Commands.TeleOp.Command;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Competition.Interleagues.Common.Robot;
import org.firstinspires.ftc.teamcode.Competition.Interleagues.Subsystems.Older.ScoringSystem;

public class LinkageCommand extends CommandBase {

    boolean up;
    ScoringSystem score;


    public LinkageCommand(ScoringSystem score, /*//Constants constants,*/ HardwareMap hardwareMap, Robot robot, boolean up){
        score = new ScoringSystem(hardwareMap, robot,  false);
        this.up = up;



    }

    @Override
    public void initialize() {
        score.setLinkagePosition(1);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
