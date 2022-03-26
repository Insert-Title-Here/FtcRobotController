package teamcode.test.Miscellanious;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import java.util.ArrayList;

import teamcode.Competition.Subsystems.ArmSystem;
import teamcode.Competition.Subsystems.EndgameSystems;
import teamcode.common.AbstractOpMode;
import teamcode.common.Constants;
import teamcode.common.MecanumDriveTrain;
import teamcode.common.Movement;
import teamcode.common.Utils;

@Autonomous(name="ArcCoast")
public class ArcCoast extends AbstractOpMode {
    MecanumDriveTrain drive;
    EndgameSystems system;
    ArmSystem arm;
    Thread armThread;
    private final int FREIGHT = 4;
    private ArrayList<Movement> warehouseSplice;
    private final double VELOCITY = 10;
    PIDFCoefficients coefficients = new PIDFCoefficients(5, 0.5, 1.0, 0.0); //2.5
    int i;
    @Override
    protected void onInitialize() {
        warehouseSplice = new ArrayList<>();
        arm = new ArmSystem(hardwareMap, false);
        system = new EndgameSystems(hardwareMap, false);
        drive = new MecanumDriveTrain(hardwareMap, true, system, arm, coefficients);
        armThread = new Thread(){
            public void run(){
                for(int i = 0; i < FREIGHT && opModeIsActive() && !isStopRequested(); i++) {
                    while(!drive.getFlagIndex(0));
                    arm.sinIntakeIndefinite(0.7, 1.0);
                    drive.setFlagIndex(0,false);
                    while(!drive.getFlagIndex(1));
                    arm.setTerminateIntake(true);
                    arm.preScore();
                    drive.setFlagIndex(1, false);
                    while (!drive.getFlagIndex(2)) ;
                    arm.raise(Constants.TOP_POSITION);
                    drive.setFlagIndex(2, false);
                    while (!drive.getFlagIndex(3));
                    arm.score();
                    Utils.sleep(250);
                    arm.retract();
                    drive.setFlagIndex(3, false);
                }

            }
        };

    }

    @Override
    protected void onStart() {
        armThread.start();
        for(int i = 0; i < FREIGHT; i++) {
            warehouseSplice.add(new Movement(6, Movement.MovementType.WALL_LOCALIZATION));
            warehouseSplice.add(new Movement(3, Movement.MovementType.WAREHOUSE_LOCALIZATION));

            //warehouseSplice.add(new Movement(1.0));
            warehouseSplice.add(new Movement(0,true));
            warehouseSplice.add(new Movement(200));

            warehouseSplice.add(new Movement(0.7, 300 + 50 * i, true));
            //warehouseSplice.add(new Movement(100 + (100 * i), 10.0, 0.0)); //increase this? new Movement(2, Movement.MovementType.WAREHOUSE_OPERATION)
            warehouseSplice.add(new Movement(700));
            warehouseSplice.add(new Movement(DcMotor.ZeroPowerBehavior.BRAKE));
            //warehouseSplice.add(new Movement(100 + (100 * i), 10, 180.0));
            warehouseSplice.add(new Movement(1, true));
            warehouseSplice.add(new Movement(6, Movement.MovementType.WALL_LOCALIZATION));
            warehouseSplice.add(new Movement(-1.0));

            warehouseSplice.add(new Movement(-6, Movement.MovementType.WAREHOUSE_LOCALIZATION));
            warehouseSplice.add(new Movement(300, 9.0, 180.0));
            //approach and score
            warehouseSplice.add(new Movement(VELOCITY, Movement.MovementType.WALL_LOCALIZATION));
            warehouseSplice.add(new Movement(200));
            warehouseSplice.add(new Movement(300, VELOCITY, -90.0));
            warehouseSplice.add(new Movement(200));
            warehouseSplice.add(new Movement(2, true));
            warehouseSplice.add(new Movement(120.0, -6.0,1600));

            warehouseSplice.add(new Movement(3, true));
            warehouseSplice.add(new Movement(200));
            warehouseSplice.add(new Movement(-20, 24));
            warehouseSplice.add(new Movement(30, Movement.MovementType.WALL_LOCALIZATION));
            //warehouseSplice.add(new Movement(120.0, 6.0,1200));
            warehouseSplice.add(new Movement(400,  2 * VELOCITY,0.0));
            drive.splicedMovement(warehouseSplice);
            warehouseSplice.clear();
        }

    }

    @Override
    protected void onStop() {

    }
}
