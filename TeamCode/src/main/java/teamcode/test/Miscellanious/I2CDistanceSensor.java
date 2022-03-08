package teamcode.test.Miscellanious;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import teamcode.common.AbstractOpMode;

@Autonomous(name="spoof")
public class I2CDistanceSensor extends AbstractOpMode {
    DistanceSensor frontDistance, backDistance;
    ColorRangeSensor front, back;
    @Override
    protected void onInitialize() {
        frontDistance = hardwareMap.get(DistanceSensor.class, "FrontColorSensorRed");
        backDistance = hardwareMap.get(DistanceSensor.class, "BackColorSensorRed");
        front = hardwareMap.get(ColorRangeSensor.class, "FrontColorSensorRed");
        back = hardwareMap.get(ColorRangeSensor.class, "BackColorSensorRed");

    }

    @Override
    protected void onStart() {
        while(opModeIsActive()){
            telemetry.addData("front", frontDistance.getDistance(DistanceUnit.INCH));
            telemetry.addData("back", backDistance.getDistance(DistanceUnit.INCH));
            telemetry.addData("front", front.getDistance(DistanceUnit.INCH));
            telemetry.addData("back", back.getDistance(DistanceUnit.INCH));
            telemetry.update();
        }
    }

    @Override
    protected void onStop() {

    }
}
