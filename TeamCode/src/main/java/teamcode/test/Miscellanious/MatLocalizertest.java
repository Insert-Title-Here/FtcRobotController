package teamcode.test.Miscellanious;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import teamcode.common.AbstractOpMode;
import teamcode.common.Localizer;
import teamcode.common.PositionStuff.Pose;
import teamcode.common.Vector2D;



@Autonomous(name="Mat")
public class MatLocalizertest extends AbstractOpMode {
    Localizer localizer;

    @Override
    protected void onInitialize() {
        localizer = new Localizer(new Pose(0,0,0),hardwareMap);
    }

    @Override
    protected void onStart() {
        localizer.start();
        while(opModeIsActive());
    }

    @Override
    protected void onStop() {
        localizer.stopThread();
    }
}
