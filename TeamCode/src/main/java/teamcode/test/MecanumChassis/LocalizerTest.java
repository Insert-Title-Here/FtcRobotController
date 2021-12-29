package teamcode.test.MecanumChassis;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import teamcode.Competition.Subsystems.ArmSystem;
import teamcode.common.AbstractOpMode;
import teamcode.common.Localizer;
import teamcode.common.MecanumDriveTrain;
import teamcode.common.PositionStuff.Pose;

@TeleOp(name="LocalizerCalibrate")
public class LocalizerTest extends AbstractOpMode {

    Localizer localizer;
    MecanumDriveTrain drive;
    ArmSystem system;

    @Override
    protected void onInitialize() {
        localizer = new Localizer(new Pose(0,0,0), hardwareMap);
        //system = new ArmSystem(hardwareMap, false);
        drive = new MecanumDriveTrain(hardwareMap, localizer);
    }

    @Override
    protected void onStart() {
        localizer.start();
        drive.rotateDistance(0.4, Math.toRadians(90));
        //drive.rotateDistance(0.4, Math.toRadians(0));
        while(opModeIsActive()){
//            system.intakeDumb(1);
            telemetry.addData("", localizer.getCurrentState());
            telemetry.addData("rv", localizer.getRightVerticalOdometerPosition());
            telemetry.addData("lv", localizer.getLeftVerticalOdometerPosition());
            telemetry.addData("h", localizer.getHorizontalOdometerPosition());
            telemetry.update();
        }
    }

    @Override
    protected void onStop() {
        localizer.writeLoggerToFile();
        localizer.stopThread();
    }
}
