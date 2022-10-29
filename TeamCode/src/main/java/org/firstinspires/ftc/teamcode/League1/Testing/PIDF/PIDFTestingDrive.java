package org.firstinspires.ftc.teamcode.League1.Testing.PIDF;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.League1.Common.Constants;
import org.firstinspires.ftc.teamcode.League1.Subsystems.MecDrive;
import org.firstinspires.ftc.teamcode.League1.Subsystems.ScoringSystem2;

@Autonomous
public class PIDFTestingDrive extends LinearOpMode {
    MecDrive drive;
    ScoringSystem2 score;
    Constants constants;

    @Override
    public void runOpMode() throws InterruptedException {
        constants = new Constants();
        drive = new MecDrive(hardwareMap, true, telemetry);
        score = new ScoringSystem2(hardwareMap, constants, telemetry);

        waitForStart();

        drive.PIDPowerNoBulk(0.5, 1000);

        while(opModeIsActive()){

        }

        drive.simpleBrake();
    }
}
