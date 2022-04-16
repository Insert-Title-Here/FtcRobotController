package teamcode.Competition.Autos.MecanumAutos.DEAutos.Velocity.WorldsAuto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import java.util.ArrayList;

import teamcode.Competition.Pipeline.MecanumPipeline.MecanumBarcodePipeline;
import teamcode.Competition.Subsystems.ArmSystem;
import teamcode.Competition.Subsystems.EndgameSystems;
import teamcode.common.AbstractOpMode;
import teamcode.common.Constants;
import teamcode.common.Debug;
import teamcode.common.Logger;
import teamcode.common.MecanumDriveTrain;
import teamcode.common.Movement;
import teamcode.common.Utils;

import static teamcode.Competition.Pipeline.MecanumPipeline.MecanumBarcodePipeline.BarcodePosition.CENTER;
import static teamcode.Competition.Pipeline.MecanumPipeline.MecanumBarcodePipeline.BarcodePosition.LEFT;

@Autonomous(name="multi freight blue")
public class BlueDEMultiFreight extends AbstractOpMode {
    MecanumDriveTrain drive;
    EndgameSystems system;
    ArmSystem arm;
    Thread armThread;
    private final int FREIGHT = 4;
    private ArrayList<Movement> warehouseSplice;
    private final double VELOCITY = 50;
    PIDFCoefficients coefficients = new PIDFCoefficients(10, 0.5, 1.0, 2.0); //2.5
    int globalIterator;
    MecanumBarcodePipeline.BarcodePosition position = MecanumBarcodePipeline.BarcodePosition.LEFT;

    @Override
    protected void onInitialize() {
        system = new EndgameSystems(hardwareMap, true);
        arm = new ArmSystem(hardwareMap, false);
        drive = new MecanumDriveTrain(hardwareMap, false, system, arm, coefficients);
        warehouseSplice = new ArrayList<>();
        Debug.log("nafbaeuifeaiufuias");
        armThread = new Thread(){
            public void run(){
                if(position == LEFT){
                    arm.raise(Constants.BOTTOM_POSITION);
                }else if(position == CENTER){
                    arm.raise(Constants.MEDIUM_POSITION + 4000);
                }else{
                    arm.raise(Constants.TOP_POSITION + 2000);
                }

                while(!drive.getFlagIndex(4));
                if(position == LEFT) {
//                    arm.runConveyorPos(1.0,3000);
//                    Utils.sleep(100);
                }else{
                    arm.score();
                }
                arm.runConveyor(1.0);

                Utils.sleep(250);
                arm.retract();
                for(int i = 0; i < FREIGHT && opModeIsActive() && !isStopRequested(); i++) {
                    while(!drive.getFlagIndex(0));
                    arm.preScoreMultiFreight(drive.getCurrenElement());
                    arm.intakeDumb(-1.0);
                    drive.setFlagIndex(0, false);
//                    while(!drive.getFlagIndex(5));
//                    arm.preScoreMultiFreight(drive.getCurrenElement());
//                    drive.setFlagIndex(5, false);
                    while (!drive.getFlagIndex(1));
                    if(i > 1){
                        arm.raise(Constants.TOP_POSITION + 2000);
                    }else {
                        arm.raise(Constants.TOP_POSITION + 2000);
                    }
                    drive.setFlagIndex(1, false);
                    while (!drive.getFlagIndex(2));
                    arm.scoreAuto();
                    Utils.sleep(250);
                    arm.retract();
                    drive.setFlagIndex(2, false);
                }

            }
        };

    }

    @Override
    protected void onStart() {
        armThread.start();
        arm.actuateWinchStop(1.0);
        drive.moveDistanceDEVelocity(800, 45, 2 * VELOCITY); // 900 -45
        Utils.sleep(100);
        drive.rotateDistanceDEUnramped(-150, 24);
        //Utils.sleep(200);


//        if(position == MecanumBarcodePipeline.BarcodePosition.LEFT){
//            Debug.log("here");
//            arm.runConveyorPos(1.0, 1500);
//        }else{
//            arm.score();
//        }
        //Utils.sleep(250);
        if(position == LEFT) {
            //Utils.sleep(300);
            arm.runConveyorPos(1.0, 1600);
            drive.setFlagIndex(4,true);
//            Utils.sleep(100);
        }else {
            drive.setFlagIndex(4, true);
            Utils.sleep(200);
        }
        drive.rotateDistanceDEUnramped(120, 24);
        Utils.sleep(100);
        drive.strafeDistanceSensor(30, Math.PI / 5.0);
        //drive.driveColorSensorWarehouse(6); //alternatively make this 1000 tics
        boolean first = true;

        for(int i = 0; i < FREIGHT; i++) {
            //warehouseSplice.add(new Movement(6, Movement.MovementType.WALL_LOCALIZATION));
            warehouseSplice.add(new Movement(1.0));
//            if(first) {
//                first = false;
                warehouseSplice.add(new Movement( 800, 2 * VELOCITY, 0.0, true));
                warehouseSplice.add(new Movement(100));
//            }
            //warehouseSplice.add(new Movement(6, Movement.MovementType.WAREHOUSE_LOCALIZATION));

            //warehouseSplice.add(new Movement(1.0));
            //warehouseSplice.add(new Movement(200));
            //warehouseSplice.add(new Movement(200)); may or may not be needed

            warehouseSplice.add(new Movement(1.0, (double)(550.0 ),0.0));
            //+ 60 *i
            //warehouseSplice.add(new Movement(100 + (100 * i), 10.0, 0.0)); //increase this? new Movement(2, Movement.MovementType.WAREHOUSE_OPERATION)
            // warehouseSplice.add(new Movement(700));
            warehouseSplice.add(new Movement(DcMotor.ZeroPowerBehavior.BRAKE));
            //warehouseSplice.add(new Movement(100 + (100 * i), 10, 180.0));
            warehouseSplice.add(new Movement(0, true));

//            warehouseSplice.add(new Movement(-1.0));


            warehouseSplice.add(new Movement(300 + 10 * i, 2 * VELOCITY, 180.0)); // 180.0
//            warehouseSplice.add(new Movement(1.0,(long)200));

            warehouseSplice.add(new Movement(-4,500));

            //warehouseSplice.add(new Movement(100));
            warehouseSplice.add(new Movement(1.0, (long)100));// change this to 100 and the arc to 1550
            //warehouseSplice.add(new Movement(100, VELOCITY, 90.0));
            //  warehouseSplice.add(new Movement(300, VELOCITY, 180.0));
            //approach and score
            //warehouseSplice.add(new Movement(200));
            //warehouseSplice.add(new Movement(300, VELOCITY, -90.0));
            //warehouseSplice.add(new Movement(300));
            //warehouseSplice.add(new Movement(100));
            // warehouseSplice.add(new Movement(100));
            warehouseSplice.add(new Movement(1, true));
//            if(i % 2 == 0){
//                warehouseSplice.add(new Movement(130.5, -30.0, 1550)); // -6, 1500
//            }else {
            warehouseSplice.add(new Movement(-131.5, 40.0, 1600)); // -6, 1500
            // }
            warehouseSplice.add(new Movement(2, true));
            warehouseSplice.add(new Movement(200));

            warehouseSplice.add(new Movement(120, 24.0));
            warehouseSplice.add(new Movement(100));
            //warehouseSplice.add(new Movement(200, 2 * VELOCITY, 180.0));
            warehouseSplice.add(new Movement(40, Math.PI / 5.0, Movement.MovementType.WALL_LOCALIZATION));
            //warehouseSplice.add(new Movement(120.0, 6.0,1200));
            //warehouseSplice.add(new Movement(400, VELOCITY,0.0));
            drive.splicedMovement(warehouseSplice);
            warehouseSplice.clear();
        }
        drive.moveDistanceDEVelocity(700, 0, 35);
      //  drive.writeLoggerToFile();
    }

    @Override
    protected void onStop() {
          drive.writeLoggerToFile();
    }
}
