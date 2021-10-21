package teamcode.test.CalibrationClasses;

import android.drm.DrmStore;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.Servo;

import teamcode.common.AbstractOpMode;
import teamcode.common.Utils;

@Autonomous(name="OdoWinch")
public class OdoWinchTest extends AbstractOpMode {

    Servo leftOdoWinch, rightOdoWinch;

    @Override
    protected void onInitialize() {
        leftOdoWinch = hardwareMap.servo.get("LeftOdoWinch");
        rightOdoWinch = hardwareMap.servo.get("RightOdoWinch");
        leftOdoWinch.setDirection(Servo.Direction.REVERSE);
    }

    @Override
    protected void onStart() {
        leftOdoWinch.setPosition(1);
        rightOdoWinch.setPosition(1);
        telemetry.addData("running to 0.5", "");
        telemetry.update();
        Utils.sleep(5000);
        telemetry.addData("running to 0", "");
        telemetry.update();
        leftOdoWinch.setPosition(0.5);
        rightOdoWinch.setPosition(0.5);
        while(opModeIsActive());

    }

    @Override
    protected void onStop() {

    }
}
