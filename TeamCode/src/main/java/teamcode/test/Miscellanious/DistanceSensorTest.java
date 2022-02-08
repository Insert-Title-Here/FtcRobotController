package teamcode.test.Miscellanious;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import teamcode.common.AbstractOpMode;
import teamcode.common.MecanumDriveTrain;

@Autonomous(name="Distance")
public class DistanceSensorTest extends AbstractOpMode {
    MecanumDriveTrain drive;


    @Override
    protected void onInitialize() {
        drive = new MecanumDriveTrain(hardwareMap, true, null);
    }

    @Override
    protected void onStart() {
        drive.strafeDistanceSensor(1.0, Math.toRadians(90));
    }

    @Override
    protected void onStop() {

    }
}
