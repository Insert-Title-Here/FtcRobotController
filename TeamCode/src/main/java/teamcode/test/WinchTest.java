package teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import teamcode.common.AbstractOpMode;

@TeleOp(name="Winch")
public class WinchTest extends AbstractOpMode {
    DcMotor winchMotor;
    Servo linkage;

    @Override
    protected void onInitialize() {
        winchMotor = hardwareMap.dcMotor.get("Winch");
        linkage = hardwareMap.servo.get("Linkage");
        linkage.setPosition(0.14);
        winchMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    @Override
    protected void onStart() {
        while(opModeIsActive()){
            while(gamepad1.left_trigger > 0.3){
                winchMotor.setPower(1);
                telemetry.addData("position", winchMotor.getCurrentPosition());
                telemetry.update();
            }
            while(gamepad1.right_trigger > 0.3){
                winchMotor.setPower(-1);
                telemetry.addData("position", winchMotor.getCurrentPosition());
                telemetry.update();
            }

            if(gamepad1.x){
                winchMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                winchMotor.setTargetPosition(6000);
                while(Math.abs(winchMotor.getTargetPosition() - winchMotor.getCurrentPosition()) > 50){
                    telemetry.addData("position", winchMotor.getCurrentPosition());
                    telemetry.update();
                    winchMotor.setPower(1);
                }
            }else if(gamepad1.a){
                winchMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                winchMotor.setTargetPosition(0);
                while(Math.abs(winchMotor.getTargetPosition() - winchMotor.getCurrentPosition()) > 50){
                    telemetry.addData("position", winchMotor.getCurrentPosition());
                    telemetry.update();
                    winchMotor.setPower(-1);
                }
            }else if(gamepad1.y){
                linkage.setPosition(0.6);
            }else if(gamepad1.b){
                linkage.setPosition(0.14);
            }
            winchMotor.setPower(0);
            telemetry.addData("position", winchMotor.getCurrentPosition());
            telemetry.update();

        }
    }

    @Override
    protected void onStop() {

    }
}
