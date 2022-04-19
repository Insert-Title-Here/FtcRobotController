package teamcode.test.CalibrationClasses;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import teamcode.common.AbstractOpMode;


@Autonomous(name="Color Sensor Calibrator")
public class ColorSensorCalibrators extends AbstractOpMode {
    NormalizedColorSensor sensor;
    private final double RESTING_R = 0.7;
    private final double RESTING_G = 0.97;
    private final double RESTING_B = 0.85;
    private final double RESTING_A = 0.85;
    private final double GAIN_ESTIMATE = 1;


    @Override
    protected void onInitialize() {
        sensor = hardwareMap.get(NormalizedColorSensor.class, "WarehouseTapeSensor");
        sensor.setGain((int)GAIN_ESTIMATE);
    }

    @Override
    protected void onStart() {
        double startTime = time;
        double deltaTime = time - startTime;
        NormalizedRGBA rgba = sensor.getNormalizedColors();
        double avgR = 0;
        double avgG = 0;
        double avgB = 0;
        double avgA = 0;
        double iterator = 0;
        while(deltaTime < 3){
            deltaTime = time - startTime;
            iterator++;
            rgba = sensor.getNormalizedColors();
            avgR += rgba.red;
            avgG += rgba.green;
            avgB += rgba.blue;
            avgA += rgba.alpha;
        }
        avgR = avgR / iterator;
        avgG = avgG / iterator;
        avgB = avgB / iterator;
        avgA = avgA / iterator;
        double targetR = (GAIN_ESTIMATE * RESTING_R) / avgR;
        double targetG = (GAIN_ESTIMATE * RESTING_G) / avgG;
        double targetB = (GAIN_ESTIMATE * RESTING_B) / avgB;
        double targetA = (GAIN_ESTIMATE * RESTING_A) / avgA;
        telemetry.addData("Sensor: ", sensor.getDeviceName());
        telemetry.addData("gain for R", targetR);
        telemetry.addData("gain for G", targetG);
        telemetry.addData("gain for B", targetB);
        telemetry.addData("gain for A", targetA);
        telemetry.update();
        while(opModeIsActive() && !isStopRequested());



    }

    @Override
    protected void onStop() {

    }
}
