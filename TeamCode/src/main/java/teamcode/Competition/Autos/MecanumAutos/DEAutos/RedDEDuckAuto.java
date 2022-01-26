package teamcode.Competition.Autos.MecanumAutos.DEAutos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import teamcode.Competition.Subsystems.ArmSystem;
import teamcode.Competition.Subsystems.EndgameSystems;
import teamcode.common.AbstractOpMode;
import teamcode.common.Constants;
import teamcode.common.Debug;
import teamcode.common.MecanumDriveTrain;

@Autonomous(name="Red DE Duck")
public class    RedDEDuckAuto extends AbstractOpMode {

    MecanumDriveTrain drive;
    ArmSystem arm;
    EndgameSystems system;
    volatile boolean[] flags;
    Thread armCommands ;
    @Override
    protected void onInitialize() {
        system = new EndgameSystems(hardwareMap, true);
        drive = new MecanumDriveTrain(hardwareMap, false, system);
        arm = new ArmSystem(hardwareMap, false);
        Debug.log("here");
        flags = new boolean[]{false, false, false, false, false};
        armCommands = new Thread(){
            public void run(){

                while(!flags[0]);
                arm.moveSlide(1.0, Constants.TOP_POSITION);
                while(!flags[1]);
                arm.score();
                try {
                    Thread.currentThread().sleep(500);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
                arm.retract();
                while(opModeIsActive());
            }
        };
    }

    @Override
    protected void onStart() {
        armCommands.start();
        arm.idleServos();
        drive.moveDistanceDE(400, 0, 0.3, 0);
        drive.rotateDistanceDE(-90, 0.3);
        drive.moveDistanceDE(1500, -90, 0.3, 0.2);
        //drive.moveDistanceDE(200, -90, 0.3, 0.2);
        flags[0] = true;
        drive.moveDistanceDE(700, -180, 0.3, 0);
        flags[1] =true;
        drive.moveDistanceDE(600, 0, 0.3, 0);
        drive.moveDistanceDE(1800, 80, 0.3, 0.2);
        drive.rotateDistanceDE(-75, 0.2);
        system.scoreDuckAuto();
        //drive.smartDuck(true);
        while(opModeIsActive());

    }

    @Override
    protected void onStop() {

    }
}
