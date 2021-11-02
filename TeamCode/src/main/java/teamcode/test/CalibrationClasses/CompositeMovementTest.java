package teamcode.test.CalibrationClasses;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import teamcode.common.AbstractOpMode;
import teamcode.common.Localizer;
import teamcode.common.MecanumDriveTrain;
import teamcode.common.Utils;
import teamcode.common.Vector2D;
import teamcode.common.WestCoastDriveTrain;

@Autonomous(name="Movement")
public class CompositeMovementTest extends AbstractOpMode {

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
        //drive.moveToPosition(new Vector2D(0,48), 12, 0);
        //Utils.sleep(5000);
        //telemetry.clear();
        //drive.moveToRotation(Math.PI, 0.5);
        while(opModeIsActive()){
            drive.setPower(gamepad1.left_stick_y, gamepad1.right_stick_x * 0.5);
            telemetry.addData("",localizer.getCurrentState().toString());
            telemetry.update();
        }
    }

    @Override
    protected void onStop() {
        localizer.writeLoggerToFile();
        localizer.stopThread();
    }
}
