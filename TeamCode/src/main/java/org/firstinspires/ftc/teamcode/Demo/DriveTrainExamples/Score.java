package org.firstinspires.ftc.teamcode.Demo.DriveTrainExamples;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Score {

    DcMotor motor;
    Servo servo;

    public Score(HardwareMap hardwareMap) {
        motor = hardwareMap.get(DcMotor.class, "motor");
        servo = hardwareMap.get(Servo.class, "servo");
    }

    public void move() {
        motor.setPower(0.3);
    }



}
