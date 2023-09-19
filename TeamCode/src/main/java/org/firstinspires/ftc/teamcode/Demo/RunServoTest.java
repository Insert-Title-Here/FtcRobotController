package org.firstinspires.ftc.teamcode.Demo;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;

public class RunServoTest extends LinearOpMode {
    private CRServo left;
    private CRServo right;
    public void runOpMode(){
        left = hardwareMap.get(CRServo.class, "left");
        right= hardwareMap.get(CRServo.class, "right");

        //initialization
        waitForStart();
        if (opModeIsActive()){
            while (opModeIsActive()){
                telemetry.update();
                left.setPower(0.8);
                right.setPower(-1);
            }
        }
    }
}
