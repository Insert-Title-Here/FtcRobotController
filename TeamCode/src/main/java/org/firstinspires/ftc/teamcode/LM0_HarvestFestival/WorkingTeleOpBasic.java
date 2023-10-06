package org.firstinspires.ftc.teamcode.LM0_HarvestFestival;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.Competition.Interleagues.Common.Vector2D;

@TeleOp(name = "WorkingTeleOpBasic")
public class WorkingTeleOpBasic extends LinearOpMode {
    private DcMotorEx fl, fr, bl, br;
    public WorkingTeleOpBasic() {

    }

    @Override
    public void runOpMode() throws InterruptedException {
        fl = hardwareMap.get(DcMotorEx.class, "fl");
        fr = hardwareMap.get(DcMotorEx.class, "fr");
        bl = hardwareMap.get(DcMotorEx.class, "bl");
        br = hardwareMap.get(DcMotorEx.class, "br");

        //robot.setShouldUpdate(false);
        fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        br.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        fl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();

        while (opModeIsActive()) {
            double leftStickX = -gamepad1.left_stick_x;
            double leftStickY = gamepad1.left_stick_y;

            if (Math.abs(leftStickX) > Math.abs(leftStickY)) {
                leftStickY = 0;

            } else if (Math.abs(leftStickY) > Math.abs(leftStickX)) {
                leftStickX = 0;

            } else {
                leftStickY = 0;
                leftStickX = 0;
            }

            setPower(new Vector2D(-leftStickX*0.5, -leftStickY*0.5), -gamepad1.right_stick_x * 0.5, false);
        }

    }

    public void setPower(double flPow, double frPow, double blPow, double brPow) {
        fl.setPower(-flPow);
        fr.setPower(frPow);
        bl.setPower(-blPow);
        br.setPower(brPow);
    }

    public void setPower(Vector2D velocity, double turnValue, boolean isSwapped) {
        turnValue = -turnValue;
        double direction = velocity.getDirection();


        double power = velocity.magnitude();
        double angle = direction + 3 * Math.PI / 4.0;
        double sin = Math.sin(angle);
        double cos = Math.cos(angle);

        if (!isSwapped) {
            setPower(-(power * sin - turnValue), -(power * cos + turnValue),
                    -(power * cos - turnValue), -(power * sin + turnValue));
        } else {
            setPower((power * sin - turnValue), (power * cos + turnValue),
                    (power * cos - turnValue), (power * sin + turnValue));
        }
    }
}
