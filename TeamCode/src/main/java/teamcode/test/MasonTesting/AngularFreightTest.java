package teamcode.test.MasonTesting;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import teamcode.Competition.Subsystems.EndgameSystems;
import teamcode.common.AbstractOpMode;
import teamcode.common.Constants;
import teamcode.common.Debug;
import teamcode.common.MecanumDriveTrain;

@Disabled
@Autonomous (name = "AngularFreightTest")
public class AngularFreightTest extends AbstractOpMode{

    MecanumDriveTrain drive;

    @Override
    protected void onInitialize() {
        drive = new MecanumDriveTrain(hardwareMap, true, null, null);
        Debug.log("Initializing");
    }

    @Override
    protected void onStart() {
        drive.moveDistanceDE(500, 90, 0.3, 0);

        while(opModeIsActive());
    }

    @Override
    protected void onStop() {

    }
}
