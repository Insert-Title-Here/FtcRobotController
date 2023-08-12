package org.firstinspires.ftc.teamcode.Testing.DistanceSensor;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;

@TeleOp
@Disabled
public class DistanceSensorTest extends LinearOpMode {
    //DistanceSensor distance;
    AnalogInput distance;
    boolean poleSeen = false;

    @Override
    public void runOpMode() throws InterruptedException {
        distance = hardwareMap.get(AnalogInput.class, "DistanceAnalog");

        waitForStart();

        while(opModeIsActive()) {
            telemetry.addData("distance", distance.getVoltage());

            /*
            if (distance.getDistance(DistanceUnit.CM) < 20) {
                telemetry.addData("pole present", true);
                poleSeen = true;
            } else {
                telemetry.addData("pole present", false);
            }

            if (poleSeen) {
                telemetry.addData("has seen a pole", true);
            } else {
                telemetry.addData("has seen a pole", false);
            }

             */

            telemetry.update();

        }

    }


}
