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
            telemetry.addData("left stick x", gamepad1.left_stick_x);
            telemetry.addData("left stick y", gamepad1.left_stick_y);
            telemetry.addData("right stick x ", gamepad1.right_stick_x);
            telemetry.update();

            drive.setPower(new Vector2D(gamepad1.left_stick_x, gamepad1.left_stick_y),  gamepad1.right_stick_x);
        }
    }

    @Override
    protected void onStop() {

    }
}
