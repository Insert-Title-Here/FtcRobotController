package teamcode.Competition.Autos.MecanumAutos.DEAutos;

import com.acmerobotics.roadrunner.drive.MecanumDrive;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import teamcode.Competition.Subsystems.ArmSystem;
import teamcode.Competition.Subsystems.EndgameSystems;
import teamcode.common.AbstractOpMode;
import teamcode.common.Debug;
import teamcode.common.MecanumDriveTrain;

@Autonomous(name="Red DE Duck")
public class RedDEDuckAuto extends AbstractOpMode {

    MecanumDriveTrain drive;
    ArmSystem arm;
    EndgameSystems system;
    @Override
    protected void onInitialize() {
        system = new EndgameSystems(hardwareMap, false);
        drive = new MecanumDriveTrain(hardwareMap, true, system);
        arm = new ArmSystem(hardwareMap, false);
        Debug.log("here");
    }

    @Override
    protected void onStart() {
        drive.moveDistanceDE(1000, 90, 0.5, 0);
    }

    @Override
    protected void onStop() {

    }
}
