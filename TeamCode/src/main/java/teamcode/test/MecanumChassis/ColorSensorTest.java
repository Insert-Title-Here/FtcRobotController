package teamcode.test.MecanumChassis;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import teamcode.common.AbstractOpMode;

@Disabled
@Autonomous(name="color")
public class ColorSensorTest extends AbstractOpMode {

    private NormalizedColorSensor sensor;

    @Override
    protected void onInitialize() {
        sensor = hardwareMap.get(NormalizedColorSensor.class, "color");
        sensor.setGain(500); //325 is tested value but i think I trust this one more

    }

    @Override
    protected void onStart() {
        while(opModeIsActive()){
            NormalizedRGBA colors = sensor.getNormalizedColors();
            telemetry.addData("r", colors.red);
            telemetry.addData("g", colors.green);
            telemetry.addData("b", colors.blue);
            telemetry.update();

        }
    }

    @Override
    protected void onStop() {

    }
}
