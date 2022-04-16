package teamcode.test.Miscellanious;

import com.acmerobotics.roadrunner.drive.MecanumDrive;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.openftc.revextensions2.ExpansionHubEx;
import org.openftc.revextensions2.RevBulkData;

import java.util.ArrayList;

import teamcode.Competition.Subsystems.ArmSystem;
import teamcode.Competition.Subsystems.EndgameSystems;
import teamcode.common.AbstractOpMode;
import teamcode.common.Constants;
import teamcode.common.Logger;
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
    PIDFCoefficients coefficients = new PIDFCoefficients(10, 0.5, 1.0, 0.0); //2.5
    int i;
    Thread loggerThread;
    Logger logger;
    int globalIterator;



    //todo pull amperage of winch motor
    @Override
    protected void onInitialize() {

        warehouseSplice = new ArrayList<>();
        arm = new ArmSystem(hardwareMap, false);
        system = new EndgameSystems(hardwareMap, false);
        drive = new MecanumDriveTrain(hardwareMap, true, system, arm, coefficients);
        logger = new Logger(new String[]{"WInchAmp.txt", "IntakeAmp.txt"});
        armThread = new Thread(){
            public void run(){
                for(int i = 0; i < FREIGHT && opModeIsActive() && !isStopRequested(); i++) {
                    while(!drive.getFlagIndex(0));
                   // arm.preScoreMulitFreight();
                    arm.intakeDumb(-1.0);
                    arm.raise(Constants.TOP_POSITION + 1000);
                    drive.setFlagIndex(0, false);
                    while (!drive.getFlagIndex(1)) ;
                    drive.setFlagIndex(1, false);
                    while (!drive.getFlagIndex(2));
                    arm.scoreAuto();
                    Utils.sleep(250);
                    arm.retract();
                    drive.setFlagIndex(2, false);
                }

            }
        };
        loggerThread = new Thread(){
            public void run(){
                while(opModeIsActive() && !isStopRequested()){
                    globalIterator++;
                    logger.writeToLogString(0, globalIterator + ", " + arm.getMilliAmpsWinch() + "\n");
                    logger.writeToLogString(1, globalIterator + ", " + arm.getMilliAmps() + "\n");

                }
            }
        };

    }

    @Override
    protected void onStart() {
        armThread.start();
        loggerThread.start();

        for(int i = 0; i < FREIGHT; i++) {
//            warehouseSplice.add(new Movement(6, Movement.MovementType.WALL_LOCALIZATION));
//            warehouseSplice.add(new Movement(3, Movement.MovementType.WAREHOUSE_LOCALIZATION));

            //warehouseSplice.add(new Movement(1.0));
            warehouseSplice.add(new Movement(1.0));
            //warehouseSplice.add(new Movement(200)); may or may not be needed

            //warehouseSplice.add(new Movement(0.7, 300 + 50 *Math.pow(i, 1.5), true));
            //warehouseSplice.add(new Movement(100 + (100 * i), 10.0, 0.0)); //increase this? new Movement(2, Movement.MovementType.WAREHOUSE_OPERATION)
           // warehouseSplice.add(new Movement(700));
            warehouseSplice.add(new Movement(DcMotor.ZeroPowerBehavior.BRAKE));
            //warehouseSplice.add(new Movement(100 + (100 * i), 10, 180.0));
            warehouseSplice.add(new Movement(0, true));
//            warehouseSplice.add(new Movement(6, Movement.MovementType.WALL_LOCALIZATION));
//            warehouseSplice.add(new Movement(-1.0));

            warehouseSplice.add(new Movement(300 + 50 * i, 2 * VELOCITY, 180.0));

//            warehouseSplice.add(new Movement(-6, Movement.MovementType.WAREHOUSE_LOCALIZATION));
            warehouseSplice.add(new Movement(300, VELOCITY, 180.0));
            //approach and score
//            warehouseSplice.add(new Movement(VELOCITY, Movement.MovementType.WALL_LOCALIZATION));
            //warehouseSplice.add(new Movement(200));
            //warehouseSplice.add(new Movement(300, VELOCITY, -90.0));
            //warehouseSplice.add(new Movement(300));
            warehouseSplice.add(new Movement(1, true));
            warehouseSplice.add(new Movement(128.0, -15.0,1500)); // -6, 1500

            warehouseSplice.add(new Movement(2, true));
            warehouseSplice.add(new Movement(200));
            warehouseSplice.add(new Movement(-20, 24));
//            warehouseSplice.add(new Movement(30, Movement.MovementType.WALL_LOCALIZATION));
            //warehouseSplice.add(new Movement(120.0, 6.0,1200));
            warehouseSplice.add(new Movement(400,  2 * VELOCITY,0.0));
            drive.splicedMovement(warehouseSplice);
            warehouseSplice.clear();
        }

    }

    @Override
    protected void onStop() {
        //logger.writeLoggerToFile();
    }
}
