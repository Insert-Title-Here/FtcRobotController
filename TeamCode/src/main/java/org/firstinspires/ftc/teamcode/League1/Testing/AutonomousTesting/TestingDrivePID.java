package org.firstinspires.ftc.teamcode.League1.Testing.AutonomousTesting;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.League1.Subsystems.MecDrive;

public class TestingDrivePID extends LinearOpMode {

    MecDrive drive;

    @Override
    public void runOpMode() throws InterruptedException {
        drive = new MecDrive(hardwareMap, false, telemetry);


        waitForStart();


        drive.PIDPowerNoBulk(0.5, 200);


    }
}
