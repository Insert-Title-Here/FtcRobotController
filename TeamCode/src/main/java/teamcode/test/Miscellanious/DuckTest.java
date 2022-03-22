package teamcode.test.Miscellanious;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.checkerframework.checker.units.qual.A;

import teamcode.Competition.Subsystems.ArmSystem;
import teamcode.Competition.Subsystems.EndgameSystems;
import teamcode.common.AbstractOpMode;
import teamcode.common.MecanumDriveTrain;

@Disabled
@Autonomous(name="duck")
public class DuckTest extends AbstractOpMode {
    EndgameSystems systems;
    MecanumDriveTrain drive;
    ArmSystem arm;
    @Override
    protected void onInitialize() {
        arm = new ArmSystem(hardwareMap, false);
        systems = new EndgameSystems(hardwareMap, false);
        drive = new MecanumDriveTrain(hardwareMap, true, systems, null);
    }

    @Override
    protected void onStart() {
        arm.intakeDumb(1.0);
        drive.duck();
        systems.scoreDuckAuto();
        while(opModeIsActive());

    }

    @Override
    protected void onStop() {

    }
}
