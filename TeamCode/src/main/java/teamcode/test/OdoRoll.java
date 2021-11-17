package teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import teamcode.common.AbstractOpMode;
import teamcode.common.Localizer;
import teamcode.common.PositionStuff.Pose;
import teamcode.common.Vector2D;


@Autonomous(name="roKFWGWNONll")
public class OdoRoll extends AbstractOpMode {

    Localizer localizer;
    @Override
    protected void onInitialize() {
        localizer = new Localizer(new Pose(0,0, 0), hardwareMap);
    }

    @Override
    protected void onStart() {
        localizer.start();
        while(opModeIsActive()){
//            telemetry.addData("",localizer.getRightVerticalOdometerPosition());
//            telemetry.addData("",localizer.getLeftVerticalOdometerPosition());
//            telemetry.addData("",localizer.getHorizontalOdometerPosition());
//            telemetry.update();
            //telemetry.addData("leftVertical: ", localizer.getLeftVerticalOdometerPosition());
            //telemetry.addData("rightVertical: ", localizer.getRightVerticalOdometerPosition());
            //telemetry.addData("horizontal: ", localizer.getHorizontalOdometerPosition());
            telemetry.addData("", localizer.getOdoEstimate().toString());
            telemetry.addData("", localizer.getPoseVelocity().toString());
            telemetry.update();
        }
        //ReadWriteFile.writeFile(AppUtil.getInstance().getSettingsFile("KalmanStateReadings.txt"), localizer.loggingString);
    }

    @Override
    protected void onStop() {

        localizer.stopThread();
    }
}
