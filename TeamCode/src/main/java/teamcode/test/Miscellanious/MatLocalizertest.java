package teamcode.test.Miscellanious;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import teamcode.common.AbstractOpMode;
import teamcode.common.Localizer;
import teamcode.common.PositionStuff.Pose;
import teamcode.common.Vector2D;


@Disabled
@Autonomous(name="Mat")
public class MatLocalizertest extends AbstractOpMode {
    Localizer localizer;

    @Override
    protected void onInitialize() {
        localizer = new Localizer(hardwareMap,new Vector2D(0,0),0,10);
    }

    @Override
    protected void onStart() {
        localizer.start();
        telemetry.addData("pose: ", localizer.getCurrentState().toString());
        telemetry.addData("iteration", localizer.getIterator());
        telemetry.update();
        while(opModeIsActive());
    }

    @Override
    protected void onStop() {
        localizer.stopThread();
    }
}
