package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import java.io.FileNotFoundException;

public class RobotThreadTest extends OpModeWrapper {

    Robot robot;

    @Override
    protected void onInitialize() throws FileNotFoundException {
        robot = new Robot(hardwareMap);
    }

    @Override
    protected void onStart() {
        robot.start();
        while(opModeIsActive());
    }

    @Override
    protected void onStop() {
        robot.stopThread();
    }
}
