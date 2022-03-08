package teamcode.test.Miscellanious;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import teamcode.Competition.Subsystems.EndgameSystems;
import teamcode.common.AbstractOpMode;
import teamcode.common.MecanumDriveTrain;

@Autonomous(name="duck")
public class DuckTest extends AbstractOpMode {
    EndgameSystems systems;
    MecanumDriveTrain drive;
    @Override
    protected void onInitialize() {
        systems = new EndgameSystems(hardwareMap, false);
        drive = new MecanumDriveTrain(hardwareMap, true, systems, null);
    }

    @Override
    protected void onStart() {
        drive.duck();
        systems.scoreDuckAuto();

    }

    @Override
    protected void onStop() {

    }
}
