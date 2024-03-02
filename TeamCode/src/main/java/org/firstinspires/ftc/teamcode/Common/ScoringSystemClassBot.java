package org.firstinspires.ftc.teamcode.Common;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class ScoringSystemClassBot {
    Servo leftGuard, rightGuard, leftGrab, rightGrab;

    DcMotorEx arm;
    public ScoringSystemClassBot(HardwareMap hardwareMap) {

        leftGuard = hardwareMap.get(Servo.class, "servo 4");
        rightGuard = hardwareMap.get(Servo.class, "servo 3");
        leftGrab = hardwareMap.get(Servo.class, "servo 2");
        rightGrab = hardwareMap.get(Servo.class, "servo 1");

        arm = hardwareMap.get(DcMotorEx.class, "motor 3");



        arm.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        arm.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        // Linkage
        //lLinkage = hardwareMap.get(ServoImplEx.class, "LeftLinkage");
        //rLinkage = hardwareMap.get(ServoImplEx.class, "RightLinkage");

        //lLinkage.setPwmRange(new PwmControl.PwmRange(500, 2500));
        //rLinkage.setPwmRange(new PwmControl.PwmRange(500, 2500));


    }

    public void goToPosGuard(double position){
        leftGuard.setPosition(position);
        rightGuard.setPosition(position);
    }

    public void goToPosGrab(double position){
        leftGrab.setPosition(position);
        rightGrab.setPosition(position);
    }

    public void goToPosArm(double position, double power){
        while(Math.abs(position - arm.getCurrentPosition()) > 3){
            arm.setPower(power);
        }
    }
    public double getArmPos(){
        return arm.getCurrentPosition();
    }
}
