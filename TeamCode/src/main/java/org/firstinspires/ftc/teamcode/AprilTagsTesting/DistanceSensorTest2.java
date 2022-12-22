package org.firstinspires.ftc.teamcode.AprilTagsTesting;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.League1.Common.Constants;

@TeleOp
public class DistanceSensorTest2 extends LinearOpMode {
    DistanceSensor distance;
    Servo servo;
    boolean poleSeen = false;

    @Override
    public void runOpMode() throws InterruptedException {
        distance = hardwareMap.get(DistanceSensor.class, "distance");
        servo = hardwareMap.get(Servo.class, "camera");
        servo.setPosition(Constants.sleeve);

        waitForStart();


        while (distance.getDistance(DistanceUnit.CM) > 20) {
            telemetry.addData("theoretical servo pos", servo.getPosition());
            telemetry.addData("distance", distance.getDistance(DistanceUnit.CM));
            telemetry.update();

            servo.setPosition(servo.getPosition() + 0.005);

            sleep(20);

        }

        while (opModeIsActive()) {

        }


    }


}
