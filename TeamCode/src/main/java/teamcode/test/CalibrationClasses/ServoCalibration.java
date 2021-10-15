package teamcode.test.CalibrationClasses;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import teamcode.common.AbstractOpMode;
import teamcode.common.Utils;

@TeleOp(name="Servo")
public class ServoCalibration extends AbstractOpMode {

    Servo servo;

    @Override
    protected void onInitialize() {
        servo = hardwareMap.servo.get("Linkage");
        servo.setPosition(0.33);
    }

    @Override
    protected void onStart() {
        servo.setPosition(0.65);
        Utils.sleep(1000);
        servo.setPosition(0.33);
        Utils.sleep(1000);
        servo.setPosition(0.68);
    }

    @Override
    protected void onStop() {

    }
}
