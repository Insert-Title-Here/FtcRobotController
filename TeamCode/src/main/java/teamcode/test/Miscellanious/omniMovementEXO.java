package teamcode.test.Miscellanious;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import teamcode.common.AbstractOpMode;
import teamcode.common.MecanumDriveTrain;


@Autonomous(name="omni")
public class omniMovementEXO extends AbstractOpMode {
    MecanumDriveTrain drive;

    @Override
    protected void onInitialize() {
        drive = new MecanumDriveTrain(hardwareMap, true, null, null, new PIDFCoefficients(5, 0.5, 1.0, 0));
    }

    @Override
    protected void onStart() {
        drive.setOmniMovement(Math.toRadians(117), 1400, -6);
    }

    @Override
    protected void onStop() {

    }
}
