package org.firstinspires.ftc.teamcode.KrishTesting;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Krish Red TeleOp")
public class TestingTeleOpRed extends TestingTeleOpFramework{
    @Override
    public void runOpMode() throws InterruptedException {
        this.blueAlliance = false;
        super.runOpMode();



    }
}
