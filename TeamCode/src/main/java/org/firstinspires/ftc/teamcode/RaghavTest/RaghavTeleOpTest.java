package org.firstinspires.ftc.teamcode.RaghavTest;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

public class RaghavTeleOpTest extends LinearOpMode {
    DcMotor motor;
    public void runOpMode() throws InterruptedException{
        motor = hardwareMap.get(DcMotor.class, "Motor");
        waitForStart();

        while(opModeIsActive()) {
            motor.setPower(gamepad1.left_stick_y);
            telemetry.addData("MotorPower", motor.getCurrentPosition());
        }
    }
}
