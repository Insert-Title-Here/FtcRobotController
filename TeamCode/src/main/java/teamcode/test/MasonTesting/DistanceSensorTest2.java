package teamcode.test.MasonTesting;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import teamcode.common.AbstractOpMode;

// CONFIG NAME : ColorSensorTest2
@Disabled
@Autonomous(name = "DistanceSensorTest2")
public class DistanceSensorTest2 extends AbstractOpMode {
    DistanceSensor d1, d2;

    @Override
    protected void onInitialize() {
        d1 = hardwareMap.get(DistanceSensor.class, "FrontDistanceSensor");
        d2 = hardwareMap.get(DistanceSensor.class, "BackDistanceSensor");
    }

    @Override
    protected void onStart() {
        while (opModeIsActive()) {
            telemetry.addData("D-F: ", d1.getDistance(DistanceUnit.INCH));
            telemetry.addData("D-B: ", d2.getDistance(DistanceUnit.INCH));
            telemetry.update();
        }
    }

    @Override
    protected void onStop() {

    }
}
