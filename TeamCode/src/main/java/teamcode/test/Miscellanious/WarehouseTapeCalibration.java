package teamcode.test.Miscellanious;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import teamcode.common.AbstractOpMode;
import teamcode.common.MecanumDriveTrain;

@Autonomous(name="tape")
public class WarehouseTapeCalibration extends AbstractOpMode {

    NormalizedColorSensor tape;
    MecanumDriveTrain drive;
    @Override
    protected void onInitialize() {
        drive = new MecanumDriveTrain(hardwareMap, true, null, null);
        // tape = hardwareMap.get(NormalizedColorSensor.class, "WarehouseTapeSensor");
     //   tape.setGain(800);
    }

    @Override
    protected void onStart() {
        drive.driveColorSensorWarehouse(4);
        while(opModeIsActive()){

        }
    }

    @Override
    protected void onStop() {

    }
}
