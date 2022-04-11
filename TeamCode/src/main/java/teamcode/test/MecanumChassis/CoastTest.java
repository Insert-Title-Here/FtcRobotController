package teamcode.test.MecanumChassis;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import teamcode.common.AbstractOpMode;
import teamcode.common.MecanumDriveTrain;

@Autonomous(name="coastee")
public class CoastTest extends AbstractOpMode {

    MecanumDriveTrain drive;
    @Override
    protected void onInitialize() {
        drive = new MecanumDriveTrain(hardwareMap, true, null, null);

    }

    @Override
    protected void onStart() {

        //drive.coastDriveEncoder(400, 1.0);
        while(opModeIsActive());
    }

    @Override
    protected void onStop() {

    }
}
