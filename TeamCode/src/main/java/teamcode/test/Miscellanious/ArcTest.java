package teamcode.test.Miscellanious;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import java.util.ArrayList;

import teamcode.Competition.Subsystems.ArmSystem;
import teamcode.Competition.Subsystems.EndgameSystems;
import teamcode.common.AbstractOpMode;
import teamcode.common.Constants;
import teamcode.common.MecanumDriveTrain;
import teamcode.common.Movement;
import teamcode.common.Utils;
import teamcode.common.Vector2D;

@Autonomous(name="Arc")
public class ArcTest extends AbstractOpMode {
    MecanumDriveTrain drive;
    EndgameSystems system;
    ArmSystem arm;
    Thread armThread;
    private final int FREIGHT = 4;
    private ArrayList<Movement> warehouseSplice;
    private final double VELOCITY = 10;
    PIDFCoefficients coefficients = new PIDFCoefficients(5, 0.5, 1.0, 0.0); //2.5


    @Override
    protected void onInitialize() {
        warehouseSplice = new ArrayList<>();
        system = new EndgameSystems(hardwareMap, false);
        arm = new ArmSystem(hardwareMap, false);
        drive = new MecanumDriveTrain(hardwareMap, true, system, arm, coefficients);

        //intake and back out
        warehouseSplice.add(new Movement(2, Movement.MovementType.WAREHOUSE_OPERATION)); //increase this?
        warehouseSplice.add(new Movement(200, 9, 180.0));
        warehouseSplice.add(new Movement(6, Movement.MovementType.WALL_LOCALIZATION));
        warehouseSplice.add(new Movement(-6, Movement.MovementType.WAREHOUSE_LOCALIZATION));
        warehouseSplice.add(new Movement(300, 9, 180.0));
//
//        //approach and score
        warehouseSplice.add(new Movement(VELOCITY, Movement.MovementType.WALL_LOCALIZATION));
        warehouseSplice.add(new Movement(200));
        warehouseSplice.add(new Movement(300, VELOCITY, -90.0));
        warehouseSplice.add(new Movement(0, true));
        warehouseSplice.add(new Movement(118.5, -6,1600));
////        warehouseSplice.add(new Movement(150, VELOCITY, -90.0));
//        warehouseSplice.add(new Movement(-135, 6));
        warehouseSplice.add(new Movement(200));
        //warehouseSplice.add(new Movement(200, VELOCITY, 180.0));
        //  warehouseSplice.add(new Movement(200));
        warehouseSplice.add(new Movement(1, true));
        warehouseSplice.add(new Movement(117.0, 6,1200));
        armThread = new Thread(){
            public void run(){
                for(int i = 0; i < FREIGHT; i++) {
                    while (!drive.getFlagIndex(0)) ;
                    arm.raise(Constants.TOP_POSITION);
                    drive.setFlagIndex(0, false);
                    while (!drive.getFlagIndex(1));
                    arm.score();
                    Utils.sleep(250);
                    arm.retract();
                    drive.setFlagIndex(1, false);
                }

            }
        };
        //drive = new MecanumDriveTrain(hardwareMap, true, system, null, new PIDFCoefficients(5, 0.5, 1.0, 0));
    }

    @Override
    protected void onStart() {
        armThread.start();
        for(int i = 0; i < FREIGHT; i++) {
            drive.splicedMovement(warehouseSplice);
        }
        while(opModeIsActive());
    }

    @Override
    protected void onStop() {

    }
}
