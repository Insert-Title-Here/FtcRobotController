package teamcode.Competition.Autos.MecanumAutos.DEAutos.Velocity.WorldsAuto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.checkerframework.checker.units.qual.A;

import teamcode.Competition.Subsystems.ArmSystem;
import teamcode.Competition.Subsystems.EndgameSystems;
import teamcode.common.AbstractOpMode;
import teamcode.common.MecanumDriveTrain;
import teamcode.common.Utils;
import teamcode.common.Vector2D;

@Autonomous(name="dook")
public class IntakeDuckNew extends AbstractOpMode {
    MecanumDriveTrain drive;
    ArmSystem system;
    EndgameSystems systems;

    @Override
    protected void onInitialize() {
        //system = new ArmSystem(hardwareMap, false);
        systems = new EndgameSystems(hardwareMap, false);
        drive = new MecanumDriveTrain(hardwareMap, true, systems, system);
    }

    private final double VELOCITY = 10;
    @Override
    protected void onStart() {
        drive.moveDistanceDEVelocity(1200, -45, VELOCITY);
        Utils.sleep(200);
        drive.rotateDistanceDE(75, 4);
        Utils.sleep(200);
        drive.strafeColorSensorWarehouse(6);
        Utils.sleep(200);
        //extend
        drive.moveDistanceDEVelocity(800, 180, VELOCITY);
        //score
        drive.moveDistanceDEVelocity(300, 90, VELOCITY / 2.0);
        Utils.sleep(200);
        drive.rotateDistanceDE(160, 4);
        Utils.sleep(200);
        drive.strafeDistanceSensor(6,0);
        Utils.sleep(200);
        drive.moveDistanceDEVelocity(350, -90, VELOCITY);
        drive.driveColorSensorWarehouse(4);
        drive.moveDistanceDEVelocity(200, 0, VELOCITY);
        drive.driveColorSensorWarehouse(4);
        drive.duck();
        systems.scoreDuckAuto();
        system.intakeDumb(1.0);


    }

    @Override
    protected void onStop() {

    }
}
