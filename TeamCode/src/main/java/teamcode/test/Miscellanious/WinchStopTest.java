package teamcode.test.Miscellanious;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import teamcode.Competition.Subsystems.ArmSystem;
import teamcode.common.AbstractOpMode;

@Disabled
@Autonomous(name="stop")
public class WinchStopTest extends AbstractOpMode {

    ArmSystem system;
    @Override
    protected void onInitialize() {
        system = new ArmSystem(hardwareMap, false);
        system.actuateWinchStop(0.55);
    }

    @Override
    protected void onStart() {
        synchronized (this){
            system.actuateWinchStop(0.0);
        }
        while(opModeIsActive() && !isStopRequested());
    }

    @Override
    protected void onStop() {

    }
}
