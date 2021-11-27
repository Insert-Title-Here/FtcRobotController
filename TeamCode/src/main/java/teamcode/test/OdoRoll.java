package teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import teamcode.common.AbstractOpMode;
import teamcode.common.Localizer;
import teamcode.common.PositionStuff.Pose;
import teamcode.common.Vector2D;
import teamcode.common.WestCoastDriveTrain;


@Autonomous(name="roll")
public class OdoRoll extends AbstractOpMode {

    Localizer localizer;
    WestCoastDriveTrain drive;
    @Override
    protected void onInitialize() {
        localizer = new Localizer(new Pose(0,0,0), hardwareMap);
        drive = new WestCoastDriveTrain(hardwareMap, localizer);
        localizer.lowerOdo();
    }

    @Override
    protected void onStart() {
        localizer.start();


        while(opModeIsActive()){
//            telemetry.addData("",localizer.getRightVerticalOdometerPosition());
//            telemetry.addData("",localizer.getLeftVerticalOdometerPosition());
//            telemetry.addData("",localizer.getHorizontalOdometerPosition());
            telemetry.addData("", localizer.getCurrentState());
            telemetry.update();
            //telemetry.addData("", localizer.getOdoEstimate().toString());
            //telemetry.addData("", localizer.getPoseVelocity().toString());
            //telemetry.update();
        }
        //ReadWriteFile.writeFile(AppUtil.getInstance().getSettingsFile("KalmanStateReadings.txt"), localizer.loggingString);
    }

    @Override
    protected void onStop() {

        localizer.stopThread();
    }
}
