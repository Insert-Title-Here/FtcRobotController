package org.firstinspires.ftc.teamcode.MecanumCode.Common;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class CapstoneArm {

    DcMotor capExtension;
    Servo grabber;

    boolean isGrabbing = false;

    public static final double CAP_SERVO_OPEN = 0.85;
    public static final double CAP_SERVO_CLOSED = 1;

    public CapstoneArm(HardwareMap hardwareMap) {
        capExtension = hardwareMap.get(DcMotor.class, "CapExtension");
        capExtension.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        capExtension.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        capExtension.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        grabber = hardwareMap.get(Servo.class, "CapstoneGrabber");
        goToPosition(0);
        grabber.setPosition(0.85);

    }

    public void setPower(double power) {
        capExtension.setPower(power);
    }

    public void goToPosition(int armPosition) {
        capExtension.setTargetPosition(armPosition);
        capExtension.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        /*
        if(armPosition == 0) {
            capExtension.setPower(0.3);
        } else {
            capExtension.setPower(0.8);
        }

         */
        capExtension.setPower(1);
        while (Math.abs(getTelemetry()[0] - armPosition) > 20) {

        }
        capExtension.setPower(0);
        capExtension.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void setGrabberPosition(double position) {
        grabber.setPosition(position);
        if (position == CAP_SERVO_OPEN) {
            isGrabbing = false;
        } else if (position == CAP_SERVO_CLOSED) {
            isGrabbing = true;
        }
    }

    public synchronized void toggleGrab() {
        if(isGrabbing) {
            setGrabberPosition(CAP_SERVO_OPEN);
            isGrabbing = false;
        } else {
            setGrabberPosition(CAP_SERVO_CLOSED);
            isGrabbing = true;
        }
    }

    public double[] getTelemetry() {
        return new double[] {capExtension.getCurrentPosition(), grabber.getPosition()};
    }

}
