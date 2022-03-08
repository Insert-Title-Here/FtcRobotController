package teamcode.test.Miscellanious;

import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import teamcode.common.AbstractOpMode;

public class WarehouseTapeCalibration extends AbstractOpMode {

    NormalizedColorSensor tape;
    @Override
    protected void onInitialize() {
        tape = hardwareMap.get(NormalizedColorSensor.class, "WarehouseTapeSensor");
        tape.setGain(800);
    }

    @Override
    protected void onStart() {
        while(opModeIsActive()){
            NormalizedRGBA rgba = tape.getNormalizedColors();
            telemetry.addData("rgba", rgba.red);
        }
    }

    @Override
    protected void onStop() {

    }
}
