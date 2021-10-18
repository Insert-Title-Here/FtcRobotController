package teamcode.offSeason;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import teamcode.common.AbstractOpMode;
import teamcode.common.Utils;
import teamcode.common.WestCoastDriveTrain;

@Autonomous(name="DriveValidator")
public class DrivetrainValidator extends AbstractOpMode {

    WestCoastDriveTrain drive;
    @Override
    protected void onInitialize() {
        drive = new WestCoastDriveTrain(hardwareMap);
    }

    @Override
    protected void onStart() {
//        drive.setPower(1, 0,0,0); //-
//        Utils.sleep(3000);
//        drive.setPower(0, 0,0,0); //
//        Utils.sleep(3000);
//        drive.setPower(0, 1,0,0); //-
//        Utils.sleep(3000);
//        drive.setPower(0, 0,0,0); //
//        Utils.sleep(3000);
//        drive.setPower(0, 0,1,0); //+
//        Utils.sleep(3000);
//        drive.setPower(0,0,0,0);
//        Utils.sleep(3000);
        drive.straightMovement(1);
        Utils.sleep(1000);
        drive.rotate(1);
        while(opModeIsActive());


    }

    @Override
    protected void onStop() {

    }
}
