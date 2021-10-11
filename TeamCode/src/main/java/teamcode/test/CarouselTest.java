package teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import teamcode.common.AbstractOpMode;

@Autonomous(name="ahfiuabuciwabiuvaiuyc")
public class CarouselTest extends AbstractOpMode {
    CRServo carousel;

    @Override
    protected void onInitialize() {
        carousel = hardwareMap.get(CRServo.class, "carousel");
    }

    @Override
    protected void onStart() {
        carousel.setDirection(DcMotorSimple.Direction.REVERSE);
        carousel.setPower(1);
        while(opModeIsActive());
    }

    @Override
    protected void onStop() {

    }
}
