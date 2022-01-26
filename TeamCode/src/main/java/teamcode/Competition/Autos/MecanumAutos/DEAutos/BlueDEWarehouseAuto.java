package teamcode.Competition.Autos.MecanumAutos.DEAutos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import teamcode.Competition.Subsystems.ArmSystem;
import teamcode.Competition.Subsystems.EndgameSystems;
import teamcode.common.AbstractOpMode;
import teamcode.common.Constants;
import teamcode.common.Debug;
import teamcode.common.MecanumDriveTrain;

@Autonomous(name="Blue DE Warehouse")
public class BlueDEWarehouseAuto extends AbstractOpMode {

    MecanumDriveTrain drive;
    ArmSystem arm;
    EndgameSystems system;
    volatile boolean[] flags;
    Thread armCommands ;
    @Override
    protected void onInitialize() {
        system = new EndgameSystems(hardwareMap, false);
        drive = new MecanumDriveTrain(hardwareMap, true, system);
        arm = new ArmSystem(hardwareMap, false);
        Debug.log("here");
        flags = new boolean[]{false, false, false, false, false};
        armCommands = new Thread(){
            public void run(){

                    while(!flags[0]);
                    arm.moveSlide( 1.0, Constants.TOP_POSITION);
                    flags[0] = false;
                    while(!flags[1]);
                    arm.score();
                    try {
                        Thread.currentThread().sleep(500);
                    } catch (InterruptedException e) {
                        e.printStackTrace();
                    }
                    arm.retract();
                    flags[1] = false;
                while(opModeIsActive()); //this was a mistake I made in my inital one too,
                // you want this here as a buffer so that the thread doesnt prematurely end vs what
                //I had which looped the threads actions and because the flags were all true there was no buffer time
            }
        };
    }

    @Override
    protected void onStart() {
        armCommands.start();
        arm.idleServos();
        /*drive.moveDistanceDE(400, 0, 0.3, 0);
        drive.rotateDistanceDE(-90, 0.3);
        drive.moveDistanceDE(1400, -90, 0.3, 0.2);
        drive.moveDistanceDE(200, -90, 0.3, 0.2);
        flags[0] = true;
        drive.moveDistanceDE(600, -180, 0.3, 0);
        flags[1] =true;
        drive.moveDistanceDE(600, 0, 0.3, 0);


        drive.goToAllianceHub;
        drive.score;
        drive.park;

         */

        drive.moveDistanceDE(400, 0, 0.3, 0);
        flags[0] = true;
        drive.rotateDistanceDE(135, 0.3);
        sleep(1000);
        drive.moveDistanceDE(350, -180, 0.3, 0);
        flags[1] = true;
        sleep(1000);
        drive.rotateDistanceDE(90, 0.3);
        sleep(1000);
        drive.moveDistanceDE(1000, -90, 0.3, 0);
        drive.moveDistanceDE(800, 0, 0.3, 0);


        while(opModeIsActive());

    }

    @Override
    protected void onStop() {

    }
}
