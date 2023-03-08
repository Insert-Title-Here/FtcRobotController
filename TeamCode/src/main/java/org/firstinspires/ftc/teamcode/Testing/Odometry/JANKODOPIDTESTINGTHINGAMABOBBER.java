package org.firstinspires.ftc.teamcode.Testing.Odometry;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Competition.MTI.MecDriveSimple;


@Autonomous
public class JANKODOPIDTESTINGTHINGAMABOBBER extends LinearOpMode {
    MecDriveSimple drive;

    @Override
    public void runOpMode() throws InterruptedException {

        drive = new MecDriveSimple(hardwareMap, telemetry);

        waitForStart();

        drive.odometryPID(0.5, 0, 0, 0, 0, 0, 10);
    }
}
