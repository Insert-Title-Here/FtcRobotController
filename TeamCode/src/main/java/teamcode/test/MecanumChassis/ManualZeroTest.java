package teamcode.test.MecanumChassis;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import teamcode.common.AbstractOpMode;
import teamcode.common.Debug;
import teamcode.common.Localizer;
import teamcode.common.PositionStuff.Pose;
import teamcode.common.Utils;
import teamcode.common.Vector2D;
import teamcode.common.WestCoastDriveTrain;

@Disabled
@Autonomous(name="Manual zero")
public class ManualZeroTest extends AbstractOpMode {
    WestCoastDriveTrain drive;
    Localizer localizer;

    @Override
    protected void onInitialize() {
        localizer = new Localizer(new Pose(0,0,0), hardwareMap);
        drive = new WestCoastDriveTrain(hardwareMap, localizer);
        localizer.lowerOdo();
    }

    @Override
    protected void onStart() {
        localizer.start();
        drive.moveToPosition(new Vector2D(-12,5), 12, 0.5, false);
        Utils.sleep(1000);
        localizer.manualZero(false);
        telemetry.addData("", localizer.getCurrentState().toString());
        telemetry.addData("", localizer.getOdoEstimate().toString());
        telemetry.update();
        Utils.sleep(1000);
        localizer.resumeUpdateCycles();
        telemetry.clear();
        while(opModeIsActive()){
            telemetry.addData("", localizer.getCurrentState().toString());
            telemetry.addData("", localizer.getOdoEstimate().toString());
            telemetry.update();
        }
    }

    @Override
    protected void onStop() {
        localizer.stopThread();

    }
}
