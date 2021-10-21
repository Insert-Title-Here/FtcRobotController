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


@TeleOp(name="tele op")
public class OfficialTeleOpScript extends AbstractOpMode {

    WestCoastDriveTrain drive; //TODO change this if necessary
    ArmSystem arm;
    Thread driveThread, driverTwoThread;
    Thread armThread;
    BNO055IMU imu;



    private static final double INTAKE_POWER = 1.0;
    private static final double SPRINT_LINEAR_MODIFIER = 1.0;
    private static final double NORMAL_LINEAR_MODIFIER = 1.0;
    private static final double SPRINT_ROTATIONAL_MODIFIER = 1.0;
    private static final double NORMAL_ROTATIONAL_MODIFIER = 0.5;
    private boolean isSprint;
    private long scoredSampleTime;
    private ScoredButtonState state;
    private PulleyState pulleyState;


    @Override
    protected void onInitialize() {
        drive = new WestCoastDriveTrain(hardwareMap);



        arm = new ArmSystem(hardwareMap, true);
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        isSprint = true;
        //Initialize IMU parameters

        state = ScoredButtonState.RETRACTING;
        pulleyState = PulleyState.RETRACTED;

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
//        driverTwoThread = new Thread(){
//            public void run(){
//                while(opModeIsActive()){
//                    driverTwoUpdate();
//                }
//            }
//        };

    }

    private void driverTwoUpdate() {


    }

    private double startTime;
    private void armUpdate() {
        if(gamepad1.right_trigger > 0.3){
            startTime = AbstractOpMode.currentOpMode().time;
            while(gamepad1.right_trigger > 0.3) {
                double elapsedTime = AbstractOpMode.currentOpMode().time - startTime;
                arm.intake(0.3 * Math.abs(Math.sin(2 * elapsedTime)) + 0.3);
                telemetry.addData("right trigger", gamepad1.right_trigger);
                telemetry.update();
            }
        }else if(gamepad1.left_trigger > 0.3){
            arm.intakeDumb(-0.3);
        }else if(gamepad1.x){
            long currentSampleTime = System.currentTimeMillis();
            if(currentSampleTime - scoredSampleTime > 1000) {
                if(pulleyState == PulleyState.EXTENDED) {
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
        }else if(gamepad1.a) {
            if(pulleyState == PulleyState.RETRACTED) {
                arm.raise(Constants.TOP_POSITION);
                pulleyState = PulleyState.EXTENDED;
            }
        } else if(gamepad1.b) {
            arm.preScore();
        } else if (gamepad1.dpad_up) {
            arm.adjustUp();
        } else if (gamepad1.dpad_down) {
            arm.adjustDown();
        } else {
            arm.intakeDumb(0);
        }
    }

    private static final double ROTATE_DPAD = 0.3;
    private static final double LINEAR_DPAD = 0.5;

    //TODO change this if necessary
    private void driveUpdate() {
      drive.setPower(gamepad1.left_stick_y, NORMAL_ROTATIONAL_MODIFIER * gamepad1.right_stick_x);

    }

    private enum ScoredButtonState{
        SCORED, RETRACTING
    }

    private enum PulleyState{
        EXTENDED, RETRACTED
    }

    @Override
    protected void onStart() {
        driveThread.start();
        armThread.start();
        //driverTwoThread.start();
        while(opModeIsActive()){
        }
    }

    @Override
    protected void onStop() {
        driveThread.interrupt();
        armThread.interrupt();
    }
}
