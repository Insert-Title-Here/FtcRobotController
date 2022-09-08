package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import java.io.FileNotFoundException;

public class RobotThreadTest extends OpModeWrapper {

    Robot robot;
    MecDrive drive;

    @Override
    protected void onInitialize() throws FileNotFoundException {
        robot = new Robot(hardwareMap);
        drive = new MecDrive(hardwareMap, robot, true, telemetry);
    }

    @Override
    protected void onStart() {
        robot.start();
        drive.moveToPosition(new Point(1000, 1000), 12);
        while(opModeIsActive());
    }

    @Override
    protected void onStop() {
        robot.stopThread();
    }
}
