package teamcode.test.MecanumChassis;

import teamcode.Competition.Subsystems.ArmSystem;
import teamcode.common.AbstractOpMode;
import teamcode.common.Constants;
import teamcode.common.Localizer;
import teamcode.common.MecanumDriveTrain;
import teamcode.common.PositionStuff.Pose;
import teamcode.common.Utils;
import teamcode.common.Vector2D;

public class DaemonClear extends AbstractOpMode {
    Localizer localizer;
    MecanumDriveTrain drive;
    Thread secondaryFunctionsThread;
    ArmSystem arm;

    /**
     * outline for a mecanum multi freight auto
     */


    private static final int FREIGHT = 1;
    private static final double VELOCITY = 15;
    private static final double OMEGA = 0.5;

    @Override
    protected void onInitialize() {
        localizer = new Localizer(new Pose(0,0,0), hardwareMap);
        drive = new MecanumDriveTrain(hardwareMap, localizer, false);
        resetSensors = false;
        intake = false;
        extend = false;
        retract = false;
        arm = new ArmSystem(hardwareMap, false);
        secondaryFunctionsThread = new Thread(){
            public void run(){
                secondaryFunctionsSequence();
            }
        };
    }

    volatile boolean resetSensors, intake, extend, retract;
    //contingency plan for 0 freight, we set this to fencepost or comment out the intake section of it
    private void secondaryFunctionsSequence(){
        for(int i = 0; i < FREIGHT; i++){
            while(!resetSensors && opModeIsActive()){}
            localizer.manualZero(false);
            resetSensors = false;

            while(!intake && opModeIsActive());
            boolean isStop = arm.intakeAuto(0.7);
            drive.setEnvironmentalTerminate(true);
            if(isStop){
                drive.seteStop(true);
            }
            intake = false;

            while(!extend && opModeIsActive());
            arm.raise(Constants.TOP_POSITION);
            extend = false;

            while(!retract && opModeIsActive());
            Utils.sleep(250);
            arm.retract();
            retract = false;

        }
    }

    @Override
    protected void onStart() {
        localizer.start();
        secondaryFunctionsThread.start();
        //starting path
        for(int i = 0; i < FREIGHT; i++) {
            resetSensors = true;
            drive.strafeDistanceSensor(VELOCITY);
            localizer.resumeUpdateCycles();
            intake = true;
            drive.moveToPosition(new Vector2D(0, 24), VELOCITY); //replace this with seekCubes() if it works
            drive.moveToPosition(new Vector2D(0, 0), -VELOCITY);
            extend = true;
            drive.moveToPosition(new Vector2D(24.5, 0), VELOCITY); //replace this with a distance sensor command?
            drive.moveToRotation(Math.toRadians(-45), OMEGA);
            arm.score();
            drive.moveToRotation(Math.toRadians(0), -OMEGA);
            retract = true;
        }

        while(opModeIsActive());
    }


    
    @Override
    protected void onStop() {

    }
}
