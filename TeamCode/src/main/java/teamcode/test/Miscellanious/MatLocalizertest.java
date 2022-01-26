package teamcode.test.Miscellanious;

import com.acmerobotics.roadrunner.drive.MecanumDrive;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import teamcode.common.AbstractOpMode;
import teamcode.common.Localizer;
import teamcode.common.MecanumDriveTrain;
import teamcode.common.PositionStuff.Pose;
import teamcode.common.Vector2D;


@Disabled
@Autonomous(name="Mat")
public class MatLocalizertest extends AbstractOpMode {
    Localizer localizer;
    MecanumDriveTrain drive;

    @Override
    protected void onInitialize() {
        localizer = new Localizer(new Pose(0,0,0),hardwareMap);
        drive = new MecanumDriveTrain(hardwareMap, localizer, false);
    }

    @Override
    protected void onStart() {
        localizer.start();
        drive.rotateDistance(Math.toRadians(90), 0.6);
        //drive.rotateDistance(Math.toRadians(0), 0.4);

        while(opModeIsActive()){
            telemetry.addData("", localizer.getCurrentState());
            telemetry.update();
        }
    }

    @Override
    protected void onStop() {
        localizer.writeLoggerToFile();
        localizer.stopThread();
    }
}
