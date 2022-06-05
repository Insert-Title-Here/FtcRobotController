package teamcode.test.Miscellanious;

import com.qualcomm.robotcore.hardware.Servo;

import teamcode.common.AbstractOpMode;
import teamcode.common.ServoInterpolator;
import teamcode.common.Utils;

public class ServoInterpolatorTest extends AbstractOpMode {
    Servo s;
    ServoInterpolator interpolator;

    @Override
    protected void onInitialize() {
        s = hardwareMap.servo.get("test");
        interpolator = new ServoInterpolator(s);
        interpolator.setConstraints(new ServoInterpolator.Constraints(1,0.4,1));
        interpolator.setTargetPosition(0.75);

    }

    @Override
    protected void onStart() {
        while(!interpolator.isAtTarget()){
            interpolator.update();
            Utils.sleep(100);
        }
    }

    @Override
    protected void onStop() {

    }
}
