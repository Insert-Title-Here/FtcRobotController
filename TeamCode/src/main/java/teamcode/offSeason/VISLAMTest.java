package teamcode.offSeason;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.Rotation;

import teamcode.common.AbstractOpMode;
import teamcode.common.Debug;
import teamcode.common.Localizer;
import teamcode.common.MecanumDriveTrain;
import teamcode.common.Utils;
import teamcode.common.Vector2D;
import teamcode.common.WestCoastDriveTrain;

@Disabled
@TeleOp(name="VISLAM Test")
public class VISLAMTest extends AbstractOpMode {
    Localizer localizer;
    WestCoastDriveTrain drive;



    @Override
    protected void onInitialize() {
        localizer = new Localizer(hardwareMap, new Vector2D(0,0), 0, 10);
        drive = new WestCoastDriveTrain(hardwareMap, localizer);

    }

    @Override
    protected void onStart() {
        //slamra.start();
        localizer.start();
        //drive.moveToPosition(new Vector2D(0,24),12, 0);
//        drive.straightMovement(0.5);
        while(opModeIsActive()){
            telemetry.addData("", localizer.getCurrentState());
            telemetry.update();
        }
    }

    @Override
    protected void onStop() {
        localizer.stopThread();
    }
}
