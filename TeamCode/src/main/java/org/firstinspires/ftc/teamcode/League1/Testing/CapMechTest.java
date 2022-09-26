package org.firstinspires.ftc.teamcode.League1.Testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.teamcode.League1.Common.Constants;
import org.firstinspires.ftc.teamcode.League1.Common.Robot;
import org.firstinspires.ftc.teamcode.League1.Common.Vector2D;
import org.firstinspires.ftc.teamcode.League1.Subsystems.MecDrive;
import org.firstinspires.ftc.teamcode.League1.Subsystems.ScoringSystem;


//TODO: figure out bulk read

@TeleOp
public class CapMechTest extends LinearOpMode {

    CRServo servo;
    @Override
    public void runOpMode() throws InterruptedException {
        servo = hardwareMap.get(CRServo.class, "CapExtend");

        waitForStart();

        while(opModeIsActive()){

            if (gamepad1.right_trigger > 0.1) {
                servo.setPower(1);
            } else if(gamepad1.left_trigger > 0.1) {
                servo.setPower(-1);
            }else{
                servo.setPower(0);
            }
        }

        servo.setPower(0);
    }
}
