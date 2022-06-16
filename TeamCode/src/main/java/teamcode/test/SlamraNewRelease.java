package teamcode.test;

import com.acmerobotics.roadrunner.geometry.Pose2d;
/*import com.spartronics4915.lib.T265Camera;
import com.spartronics4915.lib.T265Helper;
import com.spartronics4915.lib.T265Localizer;*/

import teamcode.common.AbstractOpMode;

public class SlamraNewRelease extends AbstractOpMode {

    /*
    recent changes to the FTC265 library means I need to test some stuff when I get to the school
    to test
    do I need to make it static/singleton
     */

    /*static T265Camera slamra;
    T265Localizer localizer;*/

    @Override
    protected void onInitialize() {
        /*slamra = T265Helper.getCamera(new T265Camera.OdometryInfo(new Pose2d(0,0,0), 1.0), hardwareMap.appContext);
        localizer = new T265Localizer(slamra);*/
    }

    @Override
    protected void onStart() {
        /*slamra.start();
        while(opModeIsActive()) {
            T265Camera.CameraUpdate slamraEstimate = slamra.getLastReceivedCameraUpdate();
            telemetry.addData("Pose", slamraEstimate.pose.toString());
            telemetry.addData("Velocity", slamraEstimate.velocity.toString());
            telemetry.addData("confidence", slamraEstimate.confidence);
            telemetry.update();
        }*/

    }

    @Override
    protected void onStop() {
        /*slamra.free();
        slamra.stop();*/
    }


}
