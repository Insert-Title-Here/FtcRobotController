package teamcode.offSeason;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Transform2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.spartronics4915.lib.T265Camera;

import org.firstinspires.ftc.robotcore.external.navigation.Rotation;

import teamcode.common.AbstractOpMode;
import teamcode.common.Localizer;
import teamcode.common.MecanumDriveTrain;
import teamcode.common.Utils;
import teamcode.common.Vector2D;
import teamcode.common.WestCoastDriveTrain;

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
        localizer.start();
        drive.moveToPosition(new Vector2D(0,24),12, 0);
       // drive.straightMovement(0.5);
        while(opModeIsActive());
    }

    @Override
    protected void onStop() {
        localizer.stopThread();
    }
}
