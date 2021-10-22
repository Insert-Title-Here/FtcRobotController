package teamcode.test.CalibrationClasses;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Transform2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.spartronics4915.lib.T265Camera;

import teamcode.common.AbstractOpMode;
import teamcode.common.WestCoastDriveTrain;

@TeleOp(name="VISLAM Movement")
public class VISLAMMovement extends AbstractOpMode {

    static T265Camera slamra;
    final double INCHES_TO_METERS = 0.0254;
    Transform2d transformToRobot = new Transform2d(new Translation2d(9 * INCHES_TO_METERS,-0 * INCHES_TO_METERS), new Rotation2d());
    double encoderMeasureCovariance = 1.0;
    Pose2d startingPose = new Pose2d(0,0, new Rotation2d());

    WestCoastDriveTrain driveTrain;

    @Override
    protected void onInitialize() {
        if(slamra == null){
            slamra = new T265Camera(transformToRobot, encoderMeasureCovariance, hardwareMap.appContext);
        }
        driveTrain = new WestCoastDriveTrain(hardwareMap);
    }

    @Override
    protected void onStart() {
        slamra.start();
        while(opModeIsActive()){
            T265Camera.CameraUpdate slamraUpdate = slamra.getLastReceivedCameraUpdate();
            telemetry.addData("x:", slamraUpdate.pose.getX() * INCHES_TO_METERS);
            telemetry.addData("y:", slamraUpdate.pose.getY() * INCHES_TO_METERS);
            telemetry.update();
            driveTrain.setPower(0, 0.5* gamepad1.left_stick_x);
        }
    }

    @Override
    protected void onStop() {
        slamra.stop();
    }
}
