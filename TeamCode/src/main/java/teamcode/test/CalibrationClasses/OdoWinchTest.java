package teamcode.test.CalibrationClasses;

import android.drm.DrmStore;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.Servo;

import teamcode.common.AbstractOpMode;
import teamcode.common.Utils;

@Autonomous(name="OdoWinch")
public class OdoWinchTest extends AbstractOpMode {

    Servo leftOdoWinch, rightOdoWinch;
    final double LIFT_POSITION = 0.85;

    @Override
    protected void onInitialize() {
        leftOdoWinch = hardwareMap.servo.get("LeftOdoWinch");
        rightOdoWinch = hardwareMap.servo.get("RightOdoWinch");
        leftOdoWinch.setPosition(1);
        rightOdoWinch.setPosition(1);
        //leftOdoWinch.setDirection(Servo.Direction.REVERSE);
    }

    @Override
    protected void onStart() {
        while(opModeIsActive()){
//            telemetry.addData("position", leftOdoWinch.getPosition());
//            telemetry.addData("position", rightOdoWinch.getPosition());
//            telemetry.update();
//            if(gamepad1.dpad_up){
//                leftOdoWinch.setPosition(leftOdoWinch.getPosition() + 0.02);
//                rightOdoWinch.setPosition(rightOdoWinch.getPosition() + 0.02);
//                while(gamepad1.dpad_up);
//            }else if(gamepad1.dpad_down){
//                leftOdoWinch.setPosition(leftOdoWinch.getPosition() - 0.02);
//                rightOdoWinch.setPosition(rightOdoWinch.getPosition() - 0.02);
//                while(gamepad1.dpad_down);
//            }

        }

    }

    @Override
    protected void onStop() {

    }
}
