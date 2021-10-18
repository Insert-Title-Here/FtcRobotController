package teamcode.test.CalibrationClasses;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import teamcode.common.AbstractOpMode;
import teamcode.common.Localizer;
import teamcode.common.Vector2D;

@TeleOp(name="Winch")
public class WinchTest extends AbstractOpMode {
    DcMotor winchMotor;
    Servo linkage;
    Localizer localizer;

    @Override
    protected void onInitialize() {
        winchMotor = hardwareMap.dcMotor.get("Winch");
        localizer = new Localizer(hardwareMap, new Vector2D(0,0), 0, 0.9);
        linkage = hardwareMap.servo.get("Linkage");
        linkage.setPosition(0.14);
        winchMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    @Override
    protected void onStart() {
        localizer.start();
        while(opModeIsActive()){
            while(gamepad1.left_trigger > 0.3){
                winchMotor.setPower(1);

                telemetry.update();
            }
            while(gamepad1.right_trigger > 0.3){
                winchMotor.setPower(-1);
                telemetry.update();
            }

            if(gamepad1.x){
//            runToPosition(1, -15750);
                winchMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                winchMotor.setTargetPosition(-15750);
                winchMotor.setPower(-1);
                while(Math.abs(winchMotor.getTargetPosition() - winchMotor.getCurrentPosition()) > 50){
                    telemetry.addData("position", winchMotor.getCurrentPosition());
                    telemetry.update();
                }
            }else if(gamepad1.a){
//                runToPosition(1, 0);
//                winchMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                winchMotor.setTargetPosition(0);
//                winchMotor.setPower(1);
//                while(Math.abs(winchMotor.getTargetPosition() - winchMotor.getCurrentPosition()) > 50){
//                    telemetry.addData("position", winchEncoder.getCurrentPosition());
//                    telemetry.update();
//                }
            }else if(gamepad1.y){
                linkage.setPosition(0.6);
            }else if(gamepad1.b){
                linkage.setPosition(0.14);
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
