package org.firstinspires.ftc.teamcode.Testing;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Competition.Interleagues.Common.Constants;
import org.firstinspires.ftc.teamcode.Competition.Interleagues.Common.Vector2D;
import org.firstinspires.ftc.teamcode.Competition.Interleagues.Subsystems.Used.MecDrive;
import org.firstinspires.ftc.teamcode.Competition.State.Subsystems.Current.ScoringSystemV2EpicLift;

@Disabled
@TeleOp (name = "Test Wheelie Thing")
public class WheelieBar extends LinearOpMode {

    Servo wheelieServo;

    @Override
    public void runOpMode() throws InterruptedException {

        wheelieServo = hardwareMap.get(Servo.class, "wheelie");

        wheelieServo.setPosition(Constants.wheelieRetracted);

        waitForStart();

        while(opModeIsActive()){

            if (gamepad1.a) {
                wheelieServo.setPosition(Constants.wheelieHigh);
            } else if (gamepad1.b) {
                wheelieServo.setPosition(Constants.wheelieMedium);
            } else if (gamepad1.x) {
                wheelieServo.setPosition(Constants.wheelieRetracted);
            }

        }

    }
}

