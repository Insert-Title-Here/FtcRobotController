package teamcode.test.CalibrationClasses;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Transform2d;
import com.spartronics4915.lib.T265Camera;

import teamcode.common.AbstractOpMode;

public class VISLAMMovement extends AbstractOpMode {

    static T265Camera slamra;
    Transform2d transformToRobot = new Transform2d();
    double encoderMeasureCovariance = 1.0;
    Pose2d startingPose = new Pose2d(0,0, new Rotation2d());

    @Override
    protected void onInitialize() {
        if(slamra == null){
            slamra = new T265Camera(transformToRobot, encoderMeasureCovariance, hardwareMap.appContext);

        }
    }

    @Override
    protected void onStart() {
        slamra.start();
        while(opModeIsActive()){
        }
    }

    @Override
    protected void onStop() {
        slamra.stop();
    }
}
