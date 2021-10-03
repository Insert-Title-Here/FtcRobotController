package teamcode.offSeason;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Transform2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.spartronics4915.lib.T265Camera;

import org.firstinspires.ftc.robotcore.external.navigation.Rotation;

import teamcode.common.AbstractOpMode;
import teamcode.common.MecanumDriveTrain;
import teamcode.common.Utils;
import teamcode.common.Vector2D;

@TeleOp(name="VISLAM Test")
public class VISLAMTest extends AbstractOpMode {
    MecanumDriveTrain drive;
    @Override
    protected void onInitialize() {
        drive = new MecanumDriveTrain(hardwareMap);

    }

    @Override
    protected void onStart() {
        drive.setPower(new Vector2D(0, gamepad1.right_stick_y), gamepad1.left_stick_x);
    }

    @Override
    protected void onStop() {

    }
}
