package teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;

import teamcode.common.AbstractOpMode;
import teamcode.common.Constants;
import teamcode.common.Debug;
import teamcode.common.Utils;
import teamcode.common.WestCoastDriveTrain;

import org.apache.*;

@Disabled
@Autonomous(name="Distance")
public class DistanceTravelTest extends AbstractOpMode {

    DcMotor lv, rv, h; //rv- lv+
    WestCoastDriveTrain drive;
    int rotationalTics = 25060;
    int linearTics = 22420;
    @Override
    protected void onInitialize() {
        drive = new WestCoastDriveTrain(hardwareMap);
        lv = hardwareMap.dcMotor.get(Constants.LEFT_VERTICAL_ODOMETER_NAME);
        rv = hardwareMap.dcMotor.get(Constants.RIGHT_VERTICAL_ODOMETER_NAME);
        h = hardwareMap.dcMotor.get(Constants.HORIZONTAL_ODOMETER_NAME);

    }

    @Override
    protected void onStart() {
        linear(linearTics);
    }

    private void rotate(int tics) {
        lv.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rv.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        h.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        lv.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rv.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        h.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        while(Math.abs(rv.getCurrentPosition()) < tics && opModeIsActive()){ //2* encoder value
            telemetry.addData("rv", rv.getCurrentPosition());
            telemetry.addData("lv", lv.getCurrentPosition());
            telemetry.update();

            drive.setPower(0,0.5);
        }
        drive.brake();

    }

    private void linear(int tics){
        lv.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rv.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        h.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        lv.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rv.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        h.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        while((-rv.getCurrentPosition() + lv.getCurrentPosition()) / 2.0 < tics && opModeIsActive()){

            double[] powers = drive.getMotorPowers();
            telemetry.addData("rv", -rv.getCurrentPosition());
            telemetry.addData("lv", lv.getCurrentPosition());
            telemetry.addData("fl", powers[0]);
            telemetry.addData("fr", powers[1]);
            telemetry.addData("bl", powers[2]);
            telemetry.addData("br", powers[3]);


            telemetry.update();

            drive.setPower(-0.5,0);
        }

        Debug.log("true");
        drive.brake();

    }

    @Override
    protected void onStop() {

    }
}
