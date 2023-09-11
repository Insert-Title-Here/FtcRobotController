package org.firstinspires.ftc.teamcode.Testing.Commands.TeleOp.Command;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Competition.Interleagues.Subsystems.Older.ScoringSystem;
import org.firstinspires.ftc.teamcode.Testing.Commands.Subsystems.ScoringSystemCommand;

public class FeedForwardCommand extends CommandBase {
    ScoringSystemCommand score;
    private ScoringSystem.ExtensionHeight position;
    private double power;
    GamepadEx gamepad;




    public FeedForwardCommand(ScoringSystemCommand score, /*//Constants constants,*/ HardwareMap hardwareMap, GamepadEx gamepad){
        this.score = score;
        this.gamepad = gamepad;




    }

    @Override
    public void execute() {

        if(score.isExtended()){
            score.setPower(0.23);
        }


    }

    @Override
    public boolean isFinished() {
        if(gamepad.getButton(GamepadKeys.Button.RIGHT_BUMPER)){
            score.setPower(0);
            return true;
        }else{
            return false;
        }
    }
}
