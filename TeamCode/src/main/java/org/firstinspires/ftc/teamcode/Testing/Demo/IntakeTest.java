package org.firstinspires.ftc.teamcode.Testing.Demo;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class IntakeTest extends LinearOpMode {

    CRServo lIntake;
    CRServo rIntake;

    double lPower = 1;
    double rPower = -1;

    boolean aFlag = true;
    boolean bFlag = true;
    boolean xFlag = true;
    boolean yFlag = true;

    public void runOpMode() throws InterruptedException {

        lIntake = hardwareMap.get(CRServo.class, "lIntake");
        rIntake = hardwareMap.get(CRServo.class, "rIntake");

        waitForStart();

        while (opModeIsActive()) {

            if (gamepad1.b && bFlag) {
                bFlag = false;
                rPower -= 0.05;
            }
            if (!gamepad1.b) {
                bFlag = true;
            }

            if (gamepad1.x && xFlag) {
                xFlag = false;
                rPower += 0.05;
            }
            if (!gamepad1.x) {
                xFlag = true;
            }

            if (gamepad1.y && yFlag) {
                yFlag = false;
                lPower += 0.05;
            }
            if (!gamepad1.y) {
                yFlag = true;
            }

            if (gamepad1.a && aFlag) {
                aFlag = false;
                lPower -= 0.05;
            }
            if (!gamepad1.a) {
                aFlag = true;
            }

            lIntake.setPower(lPower);
            rIntake.setPower(rPower);

            telemetry.addData("Right Power", rPower);
            telemetry.addData("Left Power", lPower);

            telemetry.update();

        }
    }
}