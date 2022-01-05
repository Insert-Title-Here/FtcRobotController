package teamcode.test.CalibrationClasses;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import teamcode.common.AbstractOpMode;
import teamcode.common.Localizer;
import teamcode.common.Vector2D;


@TeleOp(name="Winch")
public class WinchTest extends AbstractOpMode {
    DcMotor winchMotor, winchEncoder;


    @Override
    protected void onInitialize() {
        winchMotor = hardwareMap.dcMotor.get("Winch");
        winchEncoder = hardwareMap.dcMotor.get("WinchEncoder");
        winchMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        winchEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        winchEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    @Override
    protected void onStart() {
        while(opModeIsActive()) {
            while (gamepad1.left_trigger > 0.3) {
                winchMotor.setPower(0.5);
                telemetry.addData("LinearSlidePosition", winchEncoder.getCurrentPosition());
                telemetry.update();
            }
            while (gamepad1.right_trigger > 0.3) {
                winchMotor.setPower(-0.5);
                telemetry.addData("LinearSlidePosition", winchEncoder.getCurrentPosition());
                telemetry.update();
            }
            winchMotor.setPower(0);
            telemetry.update();
        }



        }
//
//    public void runToPosition(double power, int ticks){
//        winchEncoder.setTargetPosition(ticks);
//        winchEncoder.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        int error = winchEncoder.getTargetPosition() - winchEncoder.getCurrentPosition();
//        while(Math.abs(error) > 100){
//            error = winchEncoder.getTargetPosition() - winchEncoder.getCurrentPosition();
//            winchMotor.setPower(-getSign(error) * power);
//        }
//        winchMotor.setPower(0);
//    }

    private double getSign(int num) {
        if(num > 0){
            return 1;
        }else {
            return -1;
        }
    }

    @Override
    protected void onStop() {

    }
}
