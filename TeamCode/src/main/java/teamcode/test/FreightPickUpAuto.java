package teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import teamcode.Competition.Subsystems.ArmSystem;
import teamcode.common.AbstractOpMode;
import teamcode.common.Localizer;
import teamcode.common.PositionStuff.Pose;
import teamcode.common.Utils;
import teamcode.common.Vector2D;
import teamcode.common.WestCoastDriveTrain;

@Autonomous(name="freightPickUp")
public class FreightPickUpAuto extends AbstractOpMode {
    Localizer localizer;
    WestCoastDriveTrain drive;
    ArmSystem arm;


    @Override
    protected void onInitialize() {
        localizer = new Localizer(new Pose(0,0,0),hardwareMap);
        drive = new WestCoastDriveTrain(hardwareMap, localizer);
        arm = new ArmSystem(hardwareMap, false);
        localizer.lowerOdo();
    }

    @Override
    protected void onStart() {
        localizer.start();
        Utils.sleep(1000);
        localizer.liftOdo();
        drive.moveToPosition(new Vector2D(0,24), 36, 0,true);
        drive.rotateDistance(0.3, Math.toRadians(45));
        arm.lowerLinkage();
        arm.intakeDumb(0.9);
        drive.moveToPosition(new Vector2D(6,30), 12, 0,false);
        drive.moveToPosition(new Vector2D(0,24), -12, 0,false);
        arm.preScore();
        arm.intakeDumb(0);
        drive.rotateDistance(-0.3, Math.toRadians(0));
        drive.moveToPosition(new Vector2D(0,0), -36, 0,true);
        drive.rotateDistance(0.3, Math.toRadians(30));


    }

    @Override
    protected void onStop() {
        localizer.stopThread();
    }
}
