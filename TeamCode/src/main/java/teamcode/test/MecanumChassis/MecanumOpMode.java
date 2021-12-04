package teamcode.test.MecanumChassis;


import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import teamcode.common.AbstractOpMode;
import teamcode.common.MecanumDriveTrain;
import teamcode.common.Vector2D;

@TeleOp(name="Mecanum Tele Op")
public class MecanumOpMode extends AbstractOpMode {
    MecanumDriveTrain drive;

    @Override
    protected void onInitialize() {
        drive = new MecanumDriveTrain(hardwareMap);
    }

    @Override
    protected void onStart() {
        while(opModeIsActive()){
            drive.setPower(new Vector2D(gamepad1.left_stick_x, gamepad1.left_stick_y),  gamepad1.right_stick_x);
        }
    }

    @Override
    protected void onStop() {

    }
}
