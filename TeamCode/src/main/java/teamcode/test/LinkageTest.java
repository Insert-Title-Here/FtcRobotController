package teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import teamcode.common.AbstractOpMode;

@TeleOp(name="Linkage")
public class LinkageTest extends AbstractOpMode {

    Servo linkage;

    //0.55 scored 0.5 raised and 0.12 down
    @Override
    protected void onInitialize() {
        linkage = hardwareMap.servo.get("Linkage");
        //linkage.setPosition(0.2);
    }

    @Override
    protected void onStart() {
        while(opModeIsActive()){
            if(gamepad1.dpad_up){
                linkage.setPosition(linkage.getPosition() + 0.02);
                while(gamepad1.dpad_up && opModeIsActive());
            }else if(gamepad1.dpad_down){
                linkage.setPosition(linkage.getPosition() - 0.02);
                while(gamepad1.dpad_down && opModeIsActive());
            }
            telemetry.addData("linkage position", linkage.getPosition());
            telemetry.update();
        }
    }

    @Override
    protected void onStop() {

    }
}
