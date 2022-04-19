package teamcode.test.MecanumChassis;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import teamcode.common.AbstractOpMode;
import teamcode.common.Localizer;
import teamcode.common.MecanumDriveTrain;
import teamcode.common.PositionStuff.Pose;

@Disabled
@TeleOp(name="Distance sensor")
public class DistanceSensorTest extends AbstractOpMode {
    ColorRangeSensor d1, d2; //blue 0.75 //red d1 < 1.1 and d2 < 0.8
    //carousel side red is sketch


    @Override
    protected void onInitialize() {
        d1 = hardwareMap.get(ColorRangeSensor.class, "FrontColorSensorBlue");
        d2 = hardwareMap.get(ColorRangeSensor.class, "BackColorSensorBlue");


    }

    @Override
    protected void onStart() {
        //drive.strafeDistanceSensor(12);
        while(opModeIsActive()){
        telemetry.addData("", d1.getDistance(DistanceUnit.INCH));
        telemetry.addData("", d2.getDistance(DistanceUnit.INCH));
        telemetry.update();
        }
    }

    @Override
    protected void onStop() {

    }
}
