package teamcode.test.CalibrationClasses;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import teamcode.common.AbstractOpMode;

@TeleOp(name="capstoneTest")
public class CapstoneCalibration extends AbstractOpMode {

    DcMotor capstone;
    Servo capstoneServo;
    @Override
    protected void onInitialize() {
        capstone = hardwareMap.dcMotor.get("Capstone");
        capstoneServo = hardwareMap.servo.get("CapstoneServo");
        capstone.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        capstone.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        capstone.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        capstoneServo.setPosition(0.6);
    }

    @Override
    protected void onStart() {
        while(opModeIsActive()){
            if(gamepad1.dpad_up){
                capstone.setPower(0.8);
            }else if(gamepad1.dpad_down){
                capstone.setPower(-0.8);
            }else if(gamepad1.a){
                telemetry.addData("a", "");
                capstoneServo.setPosition(0);
            } else if(gamepad1.b){
                telemetry.addData("b", "");
                capstoneServo.setPosition(0.5);
            }else if(gamepad1.y){
                telemetry.addData("y", "");
                capstoneServo.setPosition(1);
            }else if(gamepad1.x){

            }else{
                capstone.setPower(0);
            }
            telemetry.addData("position", capstone.getCurrentPosition());
            telemetry.addData("servoPosition", capstoneServo.getPosition());
            telemetry.update();
        }
    }

    @Override
    protected void onStop() {

    }
}
