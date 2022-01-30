package teamcode.test.CalibrationClasses;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import teamcode.Competition.Subsystems.ArmSystem;
import teamcode.common.AbstractOpMode;

@Autonomous(name="Color sensor calibration")
public class ColorSensorTest extends AbstractOpMode {
    ArmSystem system;
    NormalizedColorSensor sensor;

    @Override
    protected void onInitialize() {
        sensor = hardwareMap.get(NormalizedColorSensor.class, "color");
        system = new ArmSystem(hardwareMap, true);
        sensor.setGain(250);
    }

    @Override
    protected void onStart() {
        while(opModeIsActive()) {
            NormalizedRGBA colors = sensor.getNormalizedColors();
            if (gamepad1.right_trigger > 0.3) {
                system.lowerLinkage();
                system.intakeDumb(1.0);
            }else{
                system.intakeDumb(0);
            }
            telemetry.addData("red", colors.red);
            telemetry.addData("green", colors.green);
            telemetry.addData("blue", colors.blue);
            telemetry.update();
        }
    }

    @Override
    protected void onStop() {

    }
}
