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
    Localizer localizer;
    Thread driveThread, driverTwoThread;
    Thread armThread;
    BNO055IMU imu;

    DcMotor leftIntake, rightIntake;


    private static final double INTAKE_POWER = 1.0;
    private static final double SPRINT_LINEAR_MODIFIER = 1.0;
    private static final double NORMAL_LINEAR_MODIFIER = 1.0;
    private static final double SPRINT_ROTATIONAL_MODIFIER = 1.0;
    private static final double NORMAL_ROTATIONAL_MODIFIER = 0.5;
    private boolean isSprint;

    @Override
    protected void onInitialize() {
        localizer = new Localizer(hardwareMap, new Vector2D(0,0), 0, 0.9);
        drive = new WestCoastDriveTrain(hardwareMap);
        arm = new ArmSystem(hardwareMap, localizer, true);
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        isSprint = true;
        //Initialize IMU parameters

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


    private void armUpdate() {
        if(gamepad1.right_trigger > 0.3){
            arm.intake(0.3);
        }else if(gamepad1.left_trigger > 0.3){

        }
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
