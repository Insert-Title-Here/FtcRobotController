package org.firstinspires.ftc.teamcode.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.OpModeWrapper;

import java.io.FileNotFoundException;


@Autonomous
public class RaganavIntakeTester extends OpModeWrapper {

    CRServo leftSpin, rightSpin;
    Servo clamp;


    @Override
    protected void onInitialize() throws FileNotFoundException {
        clamp = hardwareMap.get(Servo.class, "clamp");
        leftSpin = hardwareMap.get(CRServo.class, "lSpin");
        rightSpin = hardwareMap.get(CRServo.class, "rSpin");

        clamp.setPosition(0.3);




    }

    @Override
    protected void onStart() {

        while(opModeIsActive()){
            leftSpin.setPower(1);
            rightSpin.setPower(1);

        }

    }

    @Override
    protected void onStop() {

    }
}
