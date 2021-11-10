package teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import teamcode.common.AbstractOpMode;

@Disabled
@Autonomous(name="Color")
public class ColorSensorTest extends AbstractOpMode {

    NormalizedColorSensor sensor;
    DetectedElement element;

    @Override
    protected void onInitialize() {
        sensor = hardwareMap.get(NormalizedColorSensor.class, "color");
        element = DetectedElement.NONE;
    }

    @Override
    protected void onStart() {
        sensor.setGain(640);
        while(opModeIsActive()){
            NormalizedRGBA colors = sensor.getNormalizedColors();
            double red = colors.red;
            double green = colors.green;
            double blue = colors.blue;
            if(green > 0.9){
                if(blue > 0.9){
                    element = DetectedElement.BALL;
                }else{
                    element = DetectedElement.CUBE;
                }
            }else{
                element = DetectedElement.NONE;
            }
            telemetry.addData("element", element);
            telemetry.addData("R", red);
            telemetry.addData("G", green);
            telemetry.addData("B", blue);
            telemetry.update();
        }
    }


    private enum DetectedElement{
        BALL, CUBE, NONE
    }
    @Override
    protected void onStop() {

    }
}
