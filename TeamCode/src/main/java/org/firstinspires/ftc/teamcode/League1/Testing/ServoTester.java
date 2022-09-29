package org.firstinspires.ftc.teamcode.League1.Testing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.League1.Common.OpModeWrapper;

import java.io.FileNotFoundException;

@Autonomous
public class ServoTester extends OpModeWrapper {
    Servo rServo, lServo;
    @Override
    protected void onInitialize() throws FileNotFoundException {
        rServo = hardwareMap.get(Servo.class, "rLinkage");
        lServo = hardwareMap.get(Servo.class, "lLinkage");

        rServo.setPosition(0);
        lServo.setPosition(0);



    }

    @Override
    protected void onStart() {

        for(int i = 0; i <= 1; i += 0.1){
            rServo.setPosition(i);
            lServo.setPosition(i);
            sleep(1000);
        }
        
    }

    @Override
    protected void onStop() {
        rServo.setPosition(0);
        lServo.setPosition(0);

    }
}
