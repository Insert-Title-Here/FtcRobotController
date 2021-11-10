package teamcode.test.MasonTesting;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import teamcode.common.AbstractOpMode;

@Disabled
@TeleOp(name= "Hello World and Telemetry Test")
public class HelloWorld extends AbstractOpMode {

    // Create a servo and motor field
    DcMotor motor1;
    Servo servo1;

    @Override
    protected void onInitialize() {

        /*
        try {

        } catch ( ex) */
        motor1 = hardwareMap.dcMotor.get("Motor1");
        motor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        telemetry.addData("Motor Intialized", motor1.getMode());

        motor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor1.setTargetPosition(50);
        telemetry.addData("Motor Mode Changed", motor1.getMode());
        telemetry.addData("Motor Position Changed", motor1.getCurrentPosition());

        servo1 = hardwareMap.servo.get("Servo1");
        servo1.setPosition(20.5);
        telemetry.addLine();
        telemetry.addData("Servo Initialized", servo1.getDeviceName());
        telemetry.addLine();
        telemetry.addData("Servo Position Changed", servo1.getPosition());
    }

    @Override
    protected void onStart() {
        telemetry.addData("Hello world", "");
        telemetry.update();
        while(opModeIsActive());
    }

    @Override
    protected void onStop() {

    }
}
