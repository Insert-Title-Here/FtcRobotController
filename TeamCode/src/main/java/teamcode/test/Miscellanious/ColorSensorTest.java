package teamcode.test.Miscellanious;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import teamcode.Competition.Subsystems.ArmSystem;
import teamcode.common.AbstractOpMode;


@Autonomous(name="Color")
public class ColorSensorTest extends AbstractOpMode {

    NormalizedColorSensor sensor;
    DetectedElement element;
    ArmSystem arm;

    @Override
    protected void onInitialize() {
        sensor = hardwareMap.get(NormalizedColorSensor.class, "WarehouseTapeSensor");
        element = DetectedElement.NONE;
        arm = new ArmSystem(hardwareMap, true);
    }

    @Override
    protected void onStart() {
        sensor.setGain(480);
        while(opModeIsActive()){
            if(gamepad1.b){
                arm.preScore();
            }else if(gamepad1.a){
                arm.lowerLinkage();
            }
            if(gamepad1.right_trigger > 0.3){
                arm.intakeDumb(1.0);
            }else if(gamepad1.left_trigger > 0.3){
                arm.intakeDumb(-1.0);
            }else{
                arm.intakeDumb(0);
            }
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
