package org.firstinspires.ftc.teamcode.League1.Testing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.League1.Common.OpModeWrapper;

import java.io.FileNotFoundException;

@Autonomous
public class ServoTester extends OpModeWrapper {
    Servo servo;
    @Override
    protected void onInitialize() throws FileNotFoundException {
        servo = hardwareMap.get(Servo.class, "Linkage");
        servo.setPosition(0);



    }

    @Override
    protected void onStart() {



        servo.setPosition(1);
        sleep(4000);
        servo.setPosition(0.1);
        sleep(4000);
        servo.setPosition(0);
        sleep(4000);
        
    }

    @Override
    protected void onStop() {

    }
}
