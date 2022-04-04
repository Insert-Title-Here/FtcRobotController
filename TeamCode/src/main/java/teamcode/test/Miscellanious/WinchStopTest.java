package teamcode.test.Miscellanious;

import teamcode.Competition.Subsystems.ArmSystem;
import teamcode.common.AbstractOpMode;

public class WinchStopTest extends AbstractOpMode {

    ArmSystem system;
    @Override
    protected void onInitialize() {
        system = new ArmSystem(hardwareMap, false);
        system.actuateWinchStop(0.95);
    }

    @Override
    protected void onStart() {
        synchronized (this){
            system.actuateWinchStop(0.5);
        }
        system.setSlidePower(0.2);
        while(opModeIsActive() && !isStopRequested());
    }

    @Override
    protected void onStop() {

    }
}
