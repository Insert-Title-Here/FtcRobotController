package teamcode.test.MecanumChassis;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import teamcode.common.AbstractOpMode;

@TeleOp(name="Distance sensor")
public class DistanceSensorTest extends AbstractOpMode {
    DistanceSensor d1, d2; //blue 0.75 //red d1 < 1.1 and d2 < 0.8
    //carousel side red is sketch

    @Override
    protected void onInitialize() {
        d1 = hardwareMap.get(DistanceSensor.class, "FrontDistanceSensorRed");
        d2 = hardwareMap.get(DistanceSensor.class, "BackDistanceSensorRed");
    }

    @Override
    protected void onStart() {
        while(opModeIsActive()){
            telemetry.addData("d1", d1.getDistance(DistanceUnit.INCH));
            telemetry.addData("d2", d2.getDistance(DistanceUnit.INCH));
            telemetry.update();
        }
    }

    @Override
    protected void onStop() {

    }
}
