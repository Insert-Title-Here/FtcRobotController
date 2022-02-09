package teamcode.test.Miscellanious;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import teamcode.Competition.Subsystems.ArmSystem;
import teamcode.common.AbstractOpMode;
import teamcode.common.Constants;
import teamcode.common.MecanumDriveTrain;
import teamcode.common.Utils;

@Autonomous(name="DistanceSensor")
public class DistanceSensorTest extends AbstractOpMode {
    MecanumDriveTrain drive;
    private ArmSystem arm;
    DistanceSensor distance1, distance2;


    @Override
    protected void onInitialize() {
        arm = new ArmSystem(hardwareMap, false);
        drive = new MecanumDriveTrain(hardwareMap, false, null, arm);

    }

    @Override
    protected void onStart() {

        for(int i = 0; i < 4; i++) {
            drive.driveColorSensor(0.3);
            Utils.sleep(500);
            arm.raise(Constants.TOP_POSITION);
            arm.score();
            Utils.sleep(500);
            arm.retract();
            Utils.sleep(500);
        }

        //drive.strafeDistanceSensor(0.5, 0);
        //drive.moveDistanceDEVelocity(500, 0,12);
        //drive.strafeDistanceSensor(1.0, Math.toRadians(90));
    }

    @Override
    protected void onStop() {

    }
}
