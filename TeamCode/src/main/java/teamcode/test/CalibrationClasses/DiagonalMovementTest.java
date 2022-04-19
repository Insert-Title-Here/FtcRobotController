package teamcode.test.CalibrationClasses;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import teamcode.common.AbstractOpMode;
import teamcode.common.MecanumDriveTrain;
import teamcode.common.Utils;

@Disabled
@Autonomous(name="Diag")
public class DiagonalMovementTest extends AbstractOpMode {
    MecanumDriveTrain drive;
    private PIDFCoefficients coefficients = new PIDFCoefficients(5,0.5,1.0,0);

    @Override
    protected void onInitialize() {
        drive = new MecanumDriveTrain(hardwareMap, false, null, null, coefficients);

    }

    @Override
    protected void onStart() {
        drive.moveDistanceDEVelocity(150, 90, 5.0);
        Utils.sleep(200);
        drive.moveDistanceDEVelocity(150, 0, 5.0);

    }

    @Override
    protected void onStop() {

    }
}
