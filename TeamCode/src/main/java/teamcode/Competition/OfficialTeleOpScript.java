package teamcode.Competition;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import teamcode.common.AbstractOpMode;
import teamcode.common.Debug;
import teamcode.common.MecanumDriveTrain;
import teamcode.common.Utils;
import teamcode.common.Vector2D;
import teamcode.common.WestCoastDriveTrain;


@TeleOp(name="tele op")
public class OfficialTeleOpScript extends AbstractOpMode {

    WestCoastDriveTrain drive; //TODO change this if necessary
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
        drive = new WestCoastDriveTrain(hardwareMap);
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
      drive.setPower(gamepad1.left_stick_y, NORMAL_ROTATIONAL_MODIFIER * gamepad1.right_stick_x);
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
