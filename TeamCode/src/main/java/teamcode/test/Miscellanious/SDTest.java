package teamcode.test.Miscellanious;

import teamcode.Competition.Subsystems.EndgameSystems;
import teamcode.common.AbstractOpMode;

public class SDTest extends AbstractOpMode {
    EndgameSystems system;

    @Override
    protected void onInitialize() {
        system = new EndgameSystems(hardwareMap, false);
    }

    @Override
    protected void onStart() {
        while(opModeIsActive()){
            system.runCarousel(0.1);

        }
    }

    @Override
    protected void onStop() {

    }
}
