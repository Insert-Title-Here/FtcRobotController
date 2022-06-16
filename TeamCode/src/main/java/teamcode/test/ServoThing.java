package teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import teamcode.common.AbstractOpMode;

@TeleOp(name = "Servo Calibration")
public class ServoThing extends AbstractOpMode {
    private Servo servo;

    @Override
    protected void onInitialize() {
        servo = hardwareMap.servo.get("DebugServo");
    }

    @Override
    protected void onStart() {
        while (opModeIsActive()) {
            if (gamepad1.a) {
                servo.setPosition(0);
            }

            if (gamepad1.b) {
                servo.setPosition(1);
            }
        }
    }

    @Override
    protected void onStop() {

    }
}
