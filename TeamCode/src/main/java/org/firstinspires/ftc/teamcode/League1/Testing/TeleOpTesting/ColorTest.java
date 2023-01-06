package org.firstinspires.ftc.teamcode.League1.Testing.TeleOpTesting;

import android.media.MediaPlayer;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.checkerframework.checker.signedness.qual.Constant;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.League1.Common.Constants;
import org.firstinspires.ftc.teamcode.League1.Subsystems.ScoringSystem;
import org.firstinspires.ftc.teamcode.R;

@Disabled
@TeleOp
public class ColorTest extends LinearOpMode {
    //Constants constants;
    ColorRangeSensor distance, color;

    Servo servo;


    @Override
    public void runOpMode() throws InterruptedException {
        //constants = newConstants();
        ScoringSystem score = new ScoringSystem(hardwareMap, false);
        distance = hardwareMap.get(ColorRangeSensor.class, "distance");
        //color = hardwareMap.get(ColorRangeSensor.class, "color");

        color.setGain(300);

        //servo = hardwareMap.get(Servo.class, "Grabber");

        score.setGrabberPosition(Constants.open);

        waitForStart();

        //When alpha is above 70 then clamp then maybe once its above 200 then lift (maybe dont need the second part)

        while(opModeIsActive()){
            telemetry.addData("distance: ", distance.getDistance(DistanceUnit.CM));
            telemetry.addData("colorRed: ", color.red());
            telemetry.addData("colorBlue: ", color.blue());
            telemetry.addData("normalized Red", color.getNormalizedColors().red);
            telemetry.addData("normalized Blue", color.getNormalizedColors().blue);


            telemetry.update();

            if(gamepad1.right_trigger > 0.1){
                score.setGrabberPosition(Constants.open);
            }else if(distance.getDistance(DistanceUnit.CM) < 6.3){
                //Closed
                score.setGrabberPosition(0);
                sleep(200);
                score.setLinkagePosition(0.7);
            }else{
                score.setGrabberPosition(Constants.open);
            }

        }

    }
}
