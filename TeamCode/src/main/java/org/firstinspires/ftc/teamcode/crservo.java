package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="crservo")
public class crservo extends LinearOpMode {

    CRServo servo;
    CRServo servo2;

    @Override
    public void runOpMode() {

        servo = hardwareMap.get(CRServo.class, "servo");
        servo2 = hardwareMap.get(CRServo.class, "servo2");

        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            servo.setPower(0.8);
            servo2.setPower(0.8);
        }
    }
}