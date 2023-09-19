package org.firstinspires.ftc.teamcode.Demo;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class Intake extends LinearOpMode {

    CRServo servoLeft;
    CRServo servoRight;

    public void runOpMode() throws InterruptedException {
        servoLeft = hardwareMap.get(CRServo.class, "servoLeft");
        servoRight = hardwareMap.get(CRServo.class, "servoRight");
        waitForStart();
        double powerLeft = 0;
        boolean AFlag = true;
        boolean BFlag = true;
        double powerRight = 0;
        boolean XFlag = true;
        boolean YFlag = true;
        while (opModeIsActive()) {
            servoLeft.setPower(powerLeft);
            if (gamepad1.a && AFlag){
                powerLeft += .05;
                AFlag = false;
            }
            if(!gamepad1.a){
                AFlag = true;
            }
            if (gamepad1.b && BFlag){
                powerLeft -= .05;
                BFlag = false;
            }
            if(!gamepad1.b){
                BFlag = true;
            }
            servoRight.setPower(powerRight);
            if (gamepad1.x && XFlag){
                powerRight += .05;
                XFlag = false;
            }
            if(!gamepad1.x){
                XFlag = true;
            }
            if (gamepad1.y && YFlag){
                powerLeft -= .05;
                YFlag = false;
            }
            if(!gamepad1.y){
                YFlag = true;
            }

            telemetry.addData("Left Power", powerLeft);
            telemetry.addData("Right Power" , powerRight);

            telemetry.update();

        }
    }
}