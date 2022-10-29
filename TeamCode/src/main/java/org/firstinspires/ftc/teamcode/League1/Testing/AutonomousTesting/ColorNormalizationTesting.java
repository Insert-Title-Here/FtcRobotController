package org.firstinspires.ftc.teamcode.League1.Testing.AutonomousTesting;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;

import org.firstinspires.ftc.teamcode.League1.Common.Constants;
import org.firstinspires.ftc.teamcode.League1.Subsystems.MecDrive;
import org.firstinspires.ftc.teamcode.League1.Subsystems.ScoringSystem2;

@Autonomous
public class ColorNormalizationTesting extends LinearOpMode {
    MecDrive drive;
    Constants constants;
    ScoringSystem2 score;
    ColorRangeSensor color;


    @Override
    public void runOpMode() throws InterruptedException {
        constants = new Constants();
        drive = new MecDrive(hardwareMap, false, telemetry);
        score = new ScoringSystem2(hardwareMap, constants, telemetry);
        color = hardwareMap.get(ColorRangeSensor.class, "color");

        drive.coast();
        color.setGain(300);


        waitForStart();

        while(opModeIsActive()){
            if(color.getNormalizedColors().red > 0.38 || color.getNormalizedColors().blue > 0.8){
                score.setGrabberPosition(constants.grabbing);
            }else{
                score.setGrabberPosition(constants.open);
            }

        }
    }
}
