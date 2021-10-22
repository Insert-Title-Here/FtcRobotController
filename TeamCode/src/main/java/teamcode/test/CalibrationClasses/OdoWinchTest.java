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
        leftOdoWinch.setPosition(0);
        rightOdoWinch.setPosition(0);
        //leftOdoWinch.setDirection(Servo.Direction.REVERSE);
    }

    @Override
    protected void onStart() {
        leftOdoWinch.setPosition(LIFT_POSITION);
        rightOdoWinch.setPosition(LIFT_POSITION);
        while(opModeIsActive()){
            telemetry.addData("position", leftOdoWinch.getPosition());
            telemetry.addData("position", rightOdoWinch.getPosition());
            telemetry.update();

        }

    }

    @Override
    protected void onStop() {

    }
}
