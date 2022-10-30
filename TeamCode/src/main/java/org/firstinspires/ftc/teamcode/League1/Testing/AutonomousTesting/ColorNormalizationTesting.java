package org.firstinspires.ftc.teamcode.League1.Testing.AutonomousTesting;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.League1.Common.Constants;
import org.firstinspires.ftc.teamcode.League1.Subsystems.MecDrive;
import org.firstinspires.ftc.teamcode.League1.Subsystems.ScoringSystem2;

@Autonomous
public class ColorNormalizationTesting extends LinearOpMode {
    MecDrive drive;
    Constants constants;
    ScoringSystem2 score;
    ColorRangeSensor color, distance;


    @Override
    public void runOpMode() throws InterruptedException {
        constants = new Constants();
        drive = new MecDrive(hardwareMap, false, telemetry);
        score = new ScoringSystem2(hardwareMap, constants, telemetry);
        color = hardwareMap.get(ColorRangeSensor.class, "color");
        distance = hardwareMap.get(ColorRangeSensor.class, "distance");

        drive.coast();
        color.setGain(200);


        waitForStart();

        /*while(color.red() < 90){
            drive.setPowerAuto(0.4, MecDrive.MovementType.LDIAGONAL);
        }

        drive.simpleBrake();
*/
       color.resetDeviceConfigurationForOpMode();


        while(opModeIsActive()){
            if(color.red() > 105 || color.blue() > 200){
                score.setGrabberPosition(constants.grabbing);
            }else{
                score.setGrabberPosition(constants.open);
            }

            telemetry.addData("Nred", color.getNormalizedColors().red);
            telemetry.addData("Nblue", color.getNormalizedColors().blue);
            telemetry.addData("blue", color.blue());
            telemetry.addData("red", color.red());
            telemetry.addData("distance", distance.getDistance(DistanceUnit.CM));
            telemetry.update();


        }
    }
}
