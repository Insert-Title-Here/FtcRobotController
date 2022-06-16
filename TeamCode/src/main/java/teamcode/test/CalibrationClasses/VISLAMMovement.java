package teamcode.test.CalibrationClasses;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
/*import com.spartronics4915.lib.T265Camera;
import com.spartronics4915.lib.T265Helper;*/

import teamcode.common.AbstractOpMode;
import teamcode.common.WestCoastDriveTrain;

@Disabled
@TeleOp(name="VISLAM Movement")
public class VISLAMMovement extends AbstractOpMode {

    /*static T265Camera slamra;*/
    final double INCHES_TO_METERS = 0.0254;
    double encoderMeasureCovariance = 1.0;
    Pose2d startingPose = new Pose2d(0,0, 0);

    WestCoastDriveTrain driveTrain;

    @Override
    protected void onInitialize() {
        /*if(slamra == null){
            slamra = T265Helper.getCamera(new T265Camera.OdometryInfo(startingPose, encoderMeasureCovariance), hardwareMap.appContext);
        }*/
        driveTrain = new WestCoastDriveTrain(hardwareMap);
    }

    @Override
    protected void onStart() {
        /*slamra.start();
        while(opModeIsActive()){
            T265Camera.CameraUpdate slamraUpdate = slamra.getLastReceivedCameraUpdate();
            driveTrain.setPower(0, 0.5* gamepad1.left_stick_x);
        }*/
    }

    @Override
    protected void onStop() {
        /*slamra.stop();*/
    }
}
