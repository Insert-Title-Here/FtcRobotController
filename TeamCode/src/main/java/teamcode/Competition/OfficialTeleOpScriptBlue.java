package teamcode.Competition;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import teamcode.common.AbstractOpMode;
import teamcode.common.Constants;
import teamcode.common.Debug;
import teamcode.common.Localizer;
import teamcode.common.MecanumDriveTrain;
import teamcode.common.Utils;
import teamcode.common.Vector2D;
import teamcode.common.WestCoastDriveTrain;


@TeleOp(name="tele op BLUE")
public class OfficialTeleOpScriptBlue extends AbstractOpMode {

    WestCoastDriveTrain drive;
    ArmSystem arm;
    EndgameSystems system;
    Localizer localizer;
    Thread driveThread, driverTwoThread;
    Thread armThread;
    BNO055IMU imu;



    private static final double INTAKE_POWER = 1.0;
    private static final double SPRINT_LINEAR_MODIFIER = 1.0;
    private static final double NORMAL_LINEAR_MODIFIER = 0.8;
    private static final double SPRINT_ROTATIONAL_MODIFIER = 1.0;
    private static final double NORMAL_ROTATIONAL_MODIFIER = 0.5;
    private boolean isSprint;
    private long scoredSampleTime;

    private ScoredButtonState state;
    private PulleyState pulleyState;
    private LinkageState linkageState;
    private boolean isCarousel;


    @Override
    protected void onInitialize() {
        arm = new ArmSystem(hardwareMap, true);
        drive = new WestCoastDriveTrain(hardwareMap);
        system = new EndgameSystems(hardwareMap, true); //TODO make a copy of tele op
        localizer = new Localizer(hardwareMap, new Vector2D(0,0), 0, 10);

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        isSprint = true;
        isCarousel = false;

        localizer.liftOdo();
        //Initialize IMU parameters

        state = ScoredButtonState.RETRACTING;
        pulleyState = PulleyState.RETRACTED;
        linkageState = linkageState.RAISED;

        driveThread = new Thread(){
            public void run(){
                while(opModeIsActive()){
                    driveUpdate();
                }
            }
        };
        armThread = new Thread(){
            public void run(){
                while(opModeIsActive()){
                    armUpdate();
                }
            }
        };
        driverTwoThread = new Thread(){
            public void run(){
                while(opModeIsActive()){
                    driverTwoUpdate();
                }
            }
        };

    }

    private void driverTwoUpdate() {
        if(gamepad1.left_bumper){
            while(gamepad1.left_bumper) {
                system.runCarousel(1);
            }
        }else if(gamepad1.right_bumper){
            isCarousel = true;
            drive.setPower(-0.1,0);
            system.scoreDuck();
            drive.setPower(0,0);
            isCarousel = false;
        }else {
            system.setCapstonePower(0);
            system.runCarousel(0);
        }

//        telemetry.addData("stage", arm.getStage());
//        telemetry.update();

    }

    private double startTime;
    private void armUpdate() {
        if(gamepad1.right_trigger > 0.3){
            startTime = AbstractOpMode.currentOpMode().time;
            while(gamepad1.right_trigger > 0.3) {
                double elapsedTime = AbstractOpMode.currentOpMode().time - startTime;
                if(elapsedTime < 0.5 || linkageState == linkageState.RAISED){
                    arm.lowerLinkage();
                    linkageState = LinkageState.LOWERED;
                }else{
                    arm.intake(0.3 * Math.abs(Math.sin(2 * elapsedTime)) + 0.3, false);
                }

            }
        }else if(gamepad1.left_trigger > 0.3){
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
                arm.setWinchPower(0.5);

        } else if (gamepad1.dpad_down) {
            arm.setWinchPower(-0.5);
        } else if(gamepad1.a) {
            if (pulleyState == PulleyState.RETRACTED) {
                arm.raise(Constants.TOP_POSITION + 600);
                pulleyState = PulleyState.HIGH_GOAL;
                linkageState = LinkageState.RAISED;
            }
        }else if(gamepad1.y){
            if(pulleyState == PulleyState.RETRACTED) {
                arm.raise(Constants.MEDIUM_POSITION + 600);
                pulleyState = PulleyState.MID_GOAL;
                linkageState = linkageState.RAISED;
            }
        }else{
            arm.intakeDumb(0);
            arm.setWinchPower(0);
        }
    }

    private static final double ROTATE_DPAD = 0.3;
    private static final double LINEAR_DPAD = 0.5;

    //TODO change this if necessary
    private void driveUpdate() {
        if(!isCarousel) {
            if (gamepad1.right_stick_button) {
                drive.setPower(NORMAL_LINEAR_MODIFIER * gamepad1.left_stick_y, SPRINT_ROTATIONAL_MODIFIER * gamepad1.right_stick_x);
            } else if (gamepad1.left_stick_button) {
                drive.setPower(0.15 * gamepad1.left_stick_y, NORMAL_ROTATIONAL_MODIFIER * gamepad1.right_stick_x);
            } else {
                drive.setPower(NORMAL_LINEAR_MODIFIER * gamepad1.left_stick_y, NORMAL_ROTATIONAL_MODIFIER * gamepad1.right_stick_x);
            }
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
    protected void onStart() {
        driveThread.start();
        armThread.start();
        driverTwoThread.start();
        while(opModeIsActive()){
        }
    }

    @Override
    protected void onStop() {
        driveThread.interrupt();
        driverTwoThread.interrupt();
        armThread.interrupt();
        localizer.stopThread();
    }
}
