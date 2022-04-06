package teamcode.test.Miscellanious;

import com.qualcomm.robotcore.hardware.DcMotor;

import teamcode.common.AbstractOpMode;
import teamcode.common.Utils;

public class Test extends AbstractOpMode {
    DcMotor left, right;

    @Override
    protected void onInitialize() {
        left = hardwareMap.dcMotor.get("left");
        right = hardwareMap.dcMotor.get("right");
    }

    @Override
    protected void onStart() {
        left.setPower(0.3);
        right.setPower(0.3);
        Utils.sleep(1000);
    }

    @Override
    protected void onStop() {

    }
}
