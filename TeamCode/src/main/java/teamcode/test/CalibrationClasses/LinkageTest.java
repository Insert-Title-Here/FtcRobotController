package teamcode.test.CalibrationClasses;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import teamcode.common.AbstractOpMode;

@Disabled
@TeleOp(name="Carousel")
public class LinkageTest extends AbstractOpMode {

    Servo linkage;
    DcMotor carouselEncoder;

    //linkage, 0.38 and 0.68 lowered and raised
    //house, 0.24 intake 0.42 housed and 0.6 scored
    //carousel, ticks per rev
    @Override
    protected void onInitialize() {
        linkage = hardwareMap.servo.get("House");
        linkage.setPosition(0.6);
   }

    @Override
    protected void onStart() {
        while(opModeIsActive()){
            if(gamepad1.dpad_up){
                linkage.setPosition(linkage.getPosition() + 0.02);
                while(gamepad1.dpad_up);
            }else if(gamepad1.dpad_down){
                linkage.setPosition(linkage.getPosition() - 0.02);
                while(gamepad1.dpad_down);
            }else if(gamepad1.x){
                linkage.setPosition(0.69);

            }else if(gamepad1.a){
                linkage.setPosition(0.3);
            }
            telemetry.addData("carousel position", linkage.getPosition());
            telemetry.update();
        }
    }

    @Override
    protected void onStop() {

    }
}
