package org.firstinspires.ftc.teamcode.Testing.ColorVisionNormalization;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Competition.Interleagues.Common.Constants;
import org.firstinspires.ftc.teamcode.Competition.Interleagues.Subsystems.Used.MecDrive;
import org.firstinspires.ftc.teamcode.Competition.Interleagues.Subsystems.Used.ScoringSystem2;

@Disabled
@Autonomous
public class ColorNormalizationTesting extends LinearOpMode {
    MecDrive drive;
    //Constants constants;
    ScoringSystem2 score;
    ColorRangeSensor color, distance;


    @Override
    public void runOpMode() throws InterruptedException {
        //constants = newConstants();
        drive = new MecDrive(hardwareMap, false, telemetry, hardwareMap.get(ColorRangeSensor.class, "color"));
        score = new ScoringSystem2(hardwareMap, telemetry);
        //color = hardwareMap.get(ColorRangeSensor.class, "color");
        distance = hardwareMap.get(ColorRangeSensor.class, "distance");

        drive.coast();
        color.setGain(200);


        waitForStart();

        /*while(color.red() < 90){
            drive.setPowerAuto(0.4, MecDrive.MovementType.LDIAGONAL);
        }

        drive.simpleBrake();
*/
       //color.resetDeviceConfigurationForOpMode();

       drive.autoDiagonals(false, false, false);


        while(opModeIsActive()){
            if(color.red() > 105 || color.blue() > 200){
                score.setGrabberPosition(Constants.grabbing);
            }else{
                score.setGrabberPosition(Constants.open);
            }

            telemetry.addData("Nred", color.getNormalizedColors().red);
            telemetry.addData("Nblue", color.getNormalizedColors().blue);
            telemetry.addData("blue", color.blue());
            telemetry.addData("red", color.red());
            telemetry.addData("distance", distance.getDistance(DistanceUnit.CM));
            telemetry.addData("fl", drive.getFLEncoder());
            telemetry.addData("fr", drive.getFREncoder());
            telemetry.addData("bl", drive.getBLEncoder());
            telemetry.addData("br", drive.getBREncoder());

            telemetry.update();


        }
    }
}
