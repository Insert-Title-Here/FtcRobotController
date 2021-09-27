package teamcode.Competition;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import teamcode.common.AbstractOpMode;
import teamcode.common.Debug;
import teamcode.common.MecanumDriveTrain;
import teamcode.common.Utils;
import teamcode.common.Vector2D;



@TeleOp(name="tele op")
public class OfficialTeleOpScript extends AbstractOpMode {

    MecanumDriveTrain drive; //TODO change this if necessary
    Thread driveThread, driverTwoThread;
    Thread armThread;
    BNO055IMU imu;

    private static final double INTAKE_POWER = 1.0;
    private static final double SPRINT_LINEAR_MODIFIER = 1.0;
    private static final double NORMAL_LINEAR_MODIFIER = 1.0;
    private static final double SPRINT_ROTATIONAL_MODIFIER = 1.0;
    private static final double NORMAL_ROTATIONAL_MODIFIER = 0.5;
    private boolean isSprint;

    @Override
    protected void onInitialize() {
        drive = new MecanumDriveTrain(hardwareMap);
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        isSprint = true;
        //Initialize IMU parameters
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.RADIANS;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        //parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu.initialize(parameters);

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
                    shootUpdate();
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


    private void shootUpdate() {

    }

    private static final double ROTATE_DPAD = 0.3;
    private static final double LINEAR_DPAD = 0.5;

    //TODO change this if necessary
    private void driveUpdate() {
        if(gamepad1.right_stick_button){
            drive.setPower(new Vector2D( gamepad1.left_stick_x * NORMAL_LINEAR_MODIFIER, gamepad1.left_stick_y * NORMAL_LINEAR_MODIFIER),
                    gamepad1.right_stick_x * SPRINT_ROTATIONAL_MODIFIER);
        }else if(gamepad1.dpad_left){
            while(gamepad1.dpad_left) {
                drive.setPower(-ROTATE_DPAD, ROTATE_DPAD, -ROTATE_DPAD, ROTATE_DPAD);
            }
            drive.setPower(0,0,0,0);
        }else if(gamepad1.dpad_right){
            while(gamepad1.dpad_right){
                drive.setPower(ROTATE_DPAD, -ROTATE_DPAD,ROTATE_DPAD,-ROTATE_DPAD);
            }
            drive.setPower(0,0,0,0);
        }else if (gamepad1.dpad_up) {
            while(gamepad1.dpad_up){
                drive.setPower(LINEAR_DPAD, LINEAR_DPAD, LINEAR_DPAD, LINEAR_DPAD);
            }
            drive.zero();
        }else if(gamepad1.dpad_down) {
            while(gamepad1.dpad_down){
                drive.setPower(-LINEAR_DPAD, -LINEAR_DPAD, -LINEAR_DPAD, -LINEAR_DPAD);
            }
            drive.zero();
        }else{
            drive.setPower(new Vector2D( gamepad1.left_stick_x * NORMAL_LINEAR_MODIFIER, gamepad1.left_stick_y * NORMAL_LINEAR_MODIFIER),
                    gamepad1.right_stick_x * NORMAL_ROTATIONAL_MODIFIER);
        }

    }

    @Override
    protected void onStart() {
        //WGG.runToPosition(WobbleConstants.RETRACTED_POSITION, 0.5);
        driveThread.start();
        armThread.start();
        //driverTwoThread.start();
        while(opModeIsActive());
    }

    @Override
    protected void onStop() {

    }
}
