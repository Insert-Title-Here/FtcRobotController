package org.firstinspires.ftc.teamcode.League1.Testing;

import android.media.MediaPlayer;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.R;

@Autonomous
public class ColorTest extends LinearOpMode {
    ColorRangeSensor color;
    Servo servo;


    @Override
    public void runOpMode() throws InterruptedException {
        color = hardwareMap.get(ColorRangeSensor.class, "color");
        servo = hardwareMap.get(Servo.class, "servo");

        waitForStart();

        //When alpha is above 70 then clamp then maybe once its above 200 then lift (maybe dont need the second part)

        while(opModeIsActive()){
            telemetry.addData("red", color.red());
            telemetry.addData("blue", color.blue());
            telemetry.addData("green", color.green());
            telemetry.addData("alpha", color.alpha());
            telemetry.addData("normalized", color.getNormalizedColors());
            telemetry.update();

            if(color.alpha() > 70){
                servo.setPosition(0);
            }else{
                servo.setPosition(0.8);
            }

        }

    }
}
