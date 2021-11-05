package teamcode.test.CalibrationClasses;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import teamcode.common.AbstractOpMode;
import teamcode.common.Localizer;
import teamcode.common.Vector2D;
import teamcode.common.WestCoastDriveTrain;

@Autonomous(name="Chassis")
public class ChassisWidthCalibration extends AbstractOpMode {
    WestCoastDriveTrain drive;
    Localizer localizer;

    @Override
    protected void onInitialize() {
        localizer = new Localizer(hardwareMap, new Vector2D(0,0), 0,10);
        drive = new WestCoastDriveTrain(hardwareMap, localizer);

    }

    @Override
    protected void onStart() {
        localizer.start();
        drive.rotateDistance(0.5,Math.toRadians(180));

        while(opModeIsActive()){
            telemetry.addData("state", localizer.getCurrentState());
            telemetry.update();
        }
    }

    @Override
    protected void onStop() {
        localizer.writeLoggerToFile();
        localizer.stopThread();
    }
}
