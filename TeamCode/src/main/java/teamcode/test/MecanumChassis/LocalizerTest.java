package teamcode.test.MecanumChassis;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.openftc.revextensions2.ExpansionHubEx;

import teamcode.Competition.Subsystems.ArmSystem;
import teamcode.common.AbstractOpMode;
import teamcode.common.Localizer;
import teamcode.common.MecanumDriveTrain;
import teamcode.common.PositionStuff.Pose;
import teamcode.common.Utils;
import teamcode.common.Vector2D;

@Disabled
@TeleOp(name="LocalizerCalibrate")
public class LocalizerTest extends AbstractOpMode {

    Localizer localizer;
    MecanumDriveTrain drive;
    ArmSystem system;
    ExpansionHubEx hub;

    @Override
    protected void onInitialize() {
        localizer = new Localizer(new Pose(0,0,0), hardwareMap);
        //system = new ArmSystem(hardwareMap, false);
        drive = new MecanumDriveTrain(hardwareMap, localizer, true);
        localizer.lowerOdo();
    }

    @Override
    protected void onStart() {
        localizer.start();
        //drive.rotateDistance(Math.toRadians(180), 0.4);
        //Utils.sleep(10000);
        //drive.moveToPosition(new Vector2D(24,0), 12);
        while(opModeIsActive()){
            telemetry.addData("", localizer.getCurrentState());
            telemetry.update();
//            system.intakeDumb(1);
        }
    }

    @Override
    protected void onStop() {
        localizer.writeLoggerToFile();
        localizer.stopThread();
    }
}
