package teamcode.test.Miscellanious;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import teamcode.Competition.Subsystems.ArmSystem;
import teamcode.Competition.Subsystems.EndgameSystems;
import teamcode.common.AbstractOpMode;
import teamcode.common.MecanumDriveTrain;

@Disabled
@Autonomous(name="Stall")
public class StallReverseTest extends AbstractOpMode {

    MecanumDriveTrain drive;
    EndgameSystems system;
    ArmSystem arm;

    @Override
    protected void onInitialize() {
        arm = new ArmSystem(hardwareMap, false);
        system = new EndgameSystems(hardwareMap, false);
        drive = new MecanumDriveTrain(hardwareMap, true, system, arm);
    }

    @Override
    protected void onStart() {
        arm.lowerLinkage();
        arm.intakeReverseTest();
        while(opModeIsActive());
    }

    @Override
    protected void onStop() {

    }
}
