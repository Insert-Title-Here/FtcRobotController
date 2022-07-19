package org.firstinspires.ftc.teamcode.KrishTesting.Command.Talons.Command;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.KrishTesting.Command.Talons.TalonsEndGame;
import org.firstinspires.ftc.teamcode.KrishTesting.Command.Talons.TalonsScoringSystem;

public class CarouselCommand extends CommandBase {

    TalonsEndGame endgame;
    Gamepad gamepad;
    ElapsedTime timer;

    public CarouselCommand(TalonsEndGame endgame, Gamepad gamepad){
        this.endgame = endgame;
        timer = new ElapsedTime();
        this.gamepad = gamepad;
    }

    @Override
    public void execute() {
        endgame.runCarouselAuto();
        while(timer.seconds() < 2.5);
        endgame.brakeCarousel();
        while(timer.seconds() < 3.5);
        timer.reset();







    }

    @Override
    public boolean isFinished() {
        if(gamepad.dpad_down){
            endgame.brakeCarousel();
            return true;
        }else{
            return false;
        }
    }
}
