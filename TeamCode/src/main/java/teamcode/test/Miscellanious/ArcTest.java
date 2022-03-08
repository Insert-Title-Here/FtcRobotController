package teamcode.test.Miscellanious;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import teamcode.Competition.Subsystems.ArmSystem;
import teamcode.Competition.Subsystems.EndgameSystems;
import teamcode.common.AbstractOpMode;
import teamcode.common.Constants;
import teamcode.common.MecanumDriveTrain;
import teamcode.common.Utils;
import teamcode.common.Vector2D;

@Autonomous(name="Arc")
public class ArcTest extends AbstractOpMode {
    MecanumDriveTrain drive;
    EndgameSystems system;
    ArmSystem systems;
    Thread armThread;
    boolean[] flags = new boolean[]{false, false, false};
    private final int FREIGHT = 3;
    @Override
    protected void onInitialize() {
        systems = new ArmSystem(hardwareMap, false);
        system = new EndgameSystems(hardwareMap, false);
        armThread = new Thread(){
            public void run(){
                for(int i = 0; i < FREIGHT; i++) {
                    while (!flags[0]) ;
                    systems.raise(Constants.TOP_POSITION);
                    flags[0] = true;
                    while (!flags[1]);
                    systems.score();
                    Utils.sleep(250);
                    systems.retract();
                    flags[1] = true;
                }

            }
        };
        drive = new MecanumDriveTrain(hardwareMap, true, system, null, new PIDFCoefficients(5, 0.5, 1.0, 0));
    }

    @Override
    protected void onStart() {
        armThread.start();
        systems.intakeDumb(1.0);
        //drive.spinDuck(false);
        for(int i = 0; i < FREIGHT; i++) {
            flags[0] = true;
            drive.arcDriving(new Vector2D(-9, 0), -0.55, 500, 90);
            drive.moveDistanceDEVelocity(300, 180, 10);
            flags[1] = true;
            Utils.sleep(250);
            drive.moveDistanceDEVelocity(300, 0, 10);
            drive.arcDriving(new Vector2D(9, 0), 0.55, 500, 90);

        }
        while(opModeIsActive());
    }

    @Override
    protected void onStop() {

    }
}
