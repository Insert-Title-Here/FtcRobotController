package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


import org.firstinspires.ftc.teamcode.Common.Constants;
import org.firstinspires.ftc.teamcode.Common.MecDriveV2;
import org.firstinspires.ftc.teamcode.Common.ScoringSystem;
import org.firstinspires.ftc.teamcode.Common.Vector2D;

@Disabled
@TeleOp(name = "ServoTest")
public class ServoTest extends LinearOpMode {

    //Enums for feed forward

    Servo servo;

    @Override
    public void runOpMode() throws InterruptedException {

        servo = hardwareMap.get(Servo.class, "LeftIntakeLinkage");
        // left: 0.24 - 0.73
        // right: 0.26 - 0



        waitForStart();

        double servoPos = 0.25;

        servo.setPosition(servoPos);

        while (opModeIsActive()) {

            if (gamepad1.dpad_left) {
                servoPos += 0.01;
            } else if (gamepad1.dpad_right) {
                servoPos -= 0.01;
            }

            servo.setPosition(servoPos);

            double normalizedPosition = (servo.getPosition() - 0.26) / (0 - 0.26);

            telemetry.addData("Position", servo.getPosition());
            telemetry.addData("normalized", normalizedPosition);

            telemetry.update();

            sleep(10);

        }

    }
}
