package teamcode.test.Miscellanious;

import com.acmerobotics.roadrunner.drive.MecanumDrive;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.checkerframework.framework.qual.DefaultInUncheckedCodeFor;

import teamcode.Competition.Subsystems.ArmSystem;
import teamcode.Competition.Subsystems.EndgameSystems;
import teamcode.common.AbstractOpMode;
import teamcode.common.Debug;
import teamcode.common.MecanumDriveTrain;

@Disabled
@Autonomous(name="coast")
public class coastTest extends AbstractOpMode {

    MecanumDriveTrain drive;
    EndgameSystems system;
    ArmSystem arm;
    @Override
    protected void onInitialize() {
        system = new EndgameSystems(hardwareMap, false);
        arm = new ArmSystem(hardwareMap, false);
        drive = new MecanumDriveTrain(hardwareMap, true, system, arm);

    }

    @Override
    protected void onStart() {
        arm.lowerLinkage();
        arm.intakeDumb(1.0);
//        drive.setPower(0.2,0.2,0.2,0.2);
        drive.coastDriveEncoder(400, 1.0, 100);
        Debug.log("TERMINATED");
        while(opModeIsActive());
    }

    @Override
    protected void onStop() {

    }
}
