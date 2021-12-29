package teamcode.test.MecanumChassis;


import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import teamcode.Competition.Subsystems.ArmSystem;
import teamcode.Competition.TeleOp.OfficialTeleOpScriptBlue;
import teamcode.common.AbstractOpMode;
import teamcode.common.Constants;
import teamcode.common.Localizer;
import teamcode.common.MecanumDriveTrain;
import teamcode.common.Vector2D;

@TeleOp(name="Mecanum Tele Op")
public class MecanumOpMode extends AbstractOpMode {
    MecanumDriveTrain drive;
    ArmSystem arm;
    Thread armThread;
    Localizer localizer;

    private ScoredButtonState state;
    private PulleyState pulleyState;
    private LinkageState linkageState;
    private long scoredSampleTime;


    @Override
    protected void onInitialize() {
        drive = new MecanumDriveTrain(hardwareMap);
        arm = new ArmSystem(hardwareMap, true);

        state = ScoredButtonState.RETRACTING;
        pulleyState = PulleyState.RETRACTED;
        linkageState = linkageState.RAISED;

        localizer = new Localizer(hardwareMap, new Vector2D(0, 0), 0, 10);
        localizer.liftOdo();

        armThread = new Thread(){
            public void run(){
                while(opModeIsActive()){
                    armUpdate();
                }
            }
        };
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
                    arm.intakeDumb(0.3 * Math.abs(Math.sin(2 * elapsedTime)) + 0.7);
                }

            }
        }else if(gamepad1.a){
            //add something to move the linkage outta the way
            arm.intakeDumb(-0.6);
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
            arm.setWinchPower(-1);
        } else if(gamepad1.left_trigger > 0.3) {
            if (pulleyState == PulleyState.RETRACTED || linkageState == LinkageState.RAISED) {
                arm.raise(Constants.TOP_POSITION);
                pulleyState = PulleyState.HIGH_GOAL;
                linkageState = LinkageState.RAISED;
            }        }else if(gamepad1.dpad_right ){
            if(pulleyState == PulleyState.RETRACTED && linkageState == LinkageState.RAISED) {
                arm.raise(Constants.MEDIUM_POSITION);
                pulleyState = PulleyState.MID_GOAL;
                linkageState = linkageState.RAISED;
            }
        }else if(gamepad1.y){
            arm.score();
            arm.runConveyorPos(1.0, 2000);
            arm.idleServos();
        }else if(gamepad1.dpad_left){
            arm.resetWinchEncoder();
        }else{
            arm.intakeDumb(0);
            arm.setWinchPower(0);
        }

        telemetry.addData("slide pos", arm.getLinearSlidePosition());
        telemetry.update();
    }

    @Override
    protected void onStart() {
        armThread.start();
        while(opModeIsActive()){
            drive.setPower(new Vector2D(-gamepad1.left_stick_x, -gamepad1.left_stick_y),  0.6 * gamepad1.right_stick_x);
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
    }
}
