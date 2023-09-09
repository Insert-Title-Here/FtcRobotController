package org.firstinspires.ftc.teamcode.CenterStage;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "intakey1")
public class intakey1 extends LinearOpMode {

    DcMotor motor1;
    DcMotor motor2;
    public void runOpMode() throws InterruptedException {

        motor1 = hardwareMap.get(DcMotor.class, "motor1");
        motor2 = hardwareMap.get(DcMotor.class, "motor2");

        waitForStart();

        while (opModeIsActive()) {

            motor1.setPower(gamepad1.left_trigger);
            motor2.setPower(-gamepad1.left_trigger);

        }

    }
}