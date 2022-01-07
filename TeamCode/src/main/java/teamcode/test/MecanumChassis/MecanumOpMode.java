package teamcode.test.MecanumChassis;


import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import teamcode.Competition.Subsystems.ArmSystem;
import teamcode.Competition.Subsystems.EndgameSystems;
import teamcode.Competition.TeleOp.OfficialTeleOpScriptBlue;
import teamcode.common.AbstractOpMode;
import teamcode.common.Constants;
import teamcode.common.Debug;
import teamcode.common.Localizer;
import teamcode.common.MecanumDriveTrain;
import teamcode.common.Utils;
import teamcode.common.Vector2D;

@TeleOp(name="Mecanum Tele Op")
public class MecanumOpMode extends AbstractOpMode {
    MecanumDriveTrain drive;
    ArmSystem arm;
    Thread armThread, capThread;
    Localizer localizer;
    EndgameSystems endgameSystems;

    private ScoredButtonState state;
    private PulleyState pulleyState;
    private LinkageState linkageState;
    private long scoredSampleTime;

    int iterator;

    @Override
    protected void onInitialize() {
        drive = new MecanumDriveTrain(hardwareMap);
        localizer = new Localizer(hardwareMap, new Vector2D(0, 0), 0, 10);
        arm = new ArmSystem(hardwareMap, true);
        endgameSystems = new EndgameSystems(hardwareMap, false);

        state = ScoredButtonState.RETRACTING;
        pulleyState = PulleyState.RETRACTED;
        linkageState = linkageState.RAISED;

        localizer.liftOdo();

        armThread = new Thread(){
            public void run(){
                while(opModeIsActive()){
                    armUpdate();
                }
            }
        };

        capThread = new Thread() {
            public void run() {
                while (opModeIsActive()) {
                    capUpdate();
                }
            }
        };

        endgameSystems.zeroCap();
    }

    // For changing ranges of given variable
    private double map(double val, double in_min, double in_max, double out_min, double out_max) {
        return (val - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
    }

    // Flag variable for keeping every servo frozen until game start
    boolean capping = false;
    private void capUpdate() {

        if(gamepad2.right_trigger > 0.3 || gamepad2.left_trigger > 0.3) {
            double val = gamepad2.right_trigger - gamepad2.left_trigger;
            endgameSystems.setCapstoneExtensionPower(-val);
        }else{
            endgameSystems.setCapstoneExtensionPower(0);
        }

        double xPos = endgameSystems.getXCapPosition();
        double yPos = endgameSystems.getYCapPosition();
        endgameSystems.setXCapPosition((xPos - (map(gamepad2.left_stick_x, -1, 1, -0.0005, 0.0005))));
        endgameSystems.setYCapPosition((yPos) + map(gamepad2.right_stick_y, -1, 1, -0.001, 0.001));

        if (gamepad2.y) {
            endgameSystems.zeroCap();
        }
    }

    double startTime;
    private void armUpdate() {
        if(gamepad1.right_trigger > 0.3){
            startTime = AbstractOpMode.currentOpMode().time;
            while(gamepad1.right_trigger > 0.3) {
                double elapsedTime = AbstractOpMode.currentOpMode().time - startTime;
                if(elapsedTime < 0.5 || linkageState == linkageState.RAISED){
                    arm.lowerLinkage();
                    linkageState = LinkageState.LOWERED;
                }else{
                    arm.intakeDumb(1.0);
                }

            }
        }else if(gamepad1.a){
            //add something to move the linkage outta the way
            arm.intakeDumb(-0.8);
        }else if(gamepad1.x){
            long currentSampleTime = System.currentTimeMillis();
            if(currentSampleTime - scoredSampleTime > 200) {
                if(pulleyState != PulleyState.RETRACTED) {
                    if (state == ScoredButtonState.RETRACTING) {
                        state = ScoredButtonState.SCORED;
                        arm.score();
                    } else if (state == ScoredButtonState.SCORED) {
                        state = ScoredButtonState.RETRACTING;
                        arm.retract();
                        pulleyState = PulleyState.RETRACTED;
                    }
                }
                scoredSampleTime = System.currentTimeMillis();
            }
        } else if(gamepad1.b) {
            arm.preScore();
            linkageState = LinkageState.RAISED;
        } else if (gamepad1.dpad_up) {
            // set to full
            arm.setWinchPower(1);
        } else if (gamepad1.dpad_down) {
            arm.setWinchPower(-0.5);
        } else if(gamepad1.left_trigger > 0.3) {
            if (pulleyState == PulleyState.RETRACTED && linkageState == LinkageState.RAISED) {
                arm.raise(Constants.TOP_POSITION);
                pulleyState = PulleyState.HIGH_GOAL;
                linkageState = LinkageState.RAISED;
            }
        }else if(gamepad1.dpad_right ){
            if(pulleyState == PulleyState.RETRACTED && linkageState == LinkageState.RAISED) {
                arm.raise(Constants.MEDIUM_POSITION);
                pulleyState = PulleyState.MID_GOAL;
                linkageState = linkageState.RAISED;
            }
        }else if(gamepad1.y){
            arm.raise(Constants.BOTTOM_POSITION);
            arm.score();
            Utils.sleep(750);
            arm.idleServos();
            arm.retract();
        }else if(gamepad1.dpad_left){
            arm.resetWinchEncoder();
        }else {
            arm.intakeDumb(0);
            if (pulleyState != PulleyState.RETRACTED) {
                arm.setWinchVelocity(arm.FEEDFORWARD_V, AngleUnit.RADIANS);
            } else {
                arm.setWinchPower(0);
            }
        }


    }

    @Override
    protected void onStart() {
        armThread.start();
        capThread.start();
        while(opModeIsActive()){
            drive.setPower(new Vector2D(gamepad1.left_stick_y, -gamepad1.left_stick_x),  0.7 * gamepad1.right_stick_x);
        }
    }

    private enum ScoredButtonState{
        SCORED, RETRACTING
    }

    private enum PulleyState{
        HIGH_GOAL, MID_GOAL, RETRACTED
    }

    private enum LinkageState{
        RAISED, LOWERED
    }


    @Override
    protected void onStop() {
        armThread.interrupt();
        capThread.interrupt();
    }
}
