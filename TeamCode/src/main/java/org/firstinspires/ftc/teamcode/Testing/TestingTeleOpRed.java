package org.firstinspires.ftc.teamcode.Testing;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
@Disabled
@TeleOp(name = "Krish Red TeleOp")
public class TestingTeleOpRed extends TestingTeleOpFramework{
    @Override
    public void runOpMode() throws InterruptedException {
        this.blueAlliance = false;
        super.runOpMode();



    }
}
