package org.firstinspires.ftc.teamcode.MecanumCode.Common;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class CapstoneArm {

    DcMotor capExtension;
    Servo grabber;

    boolean isGrabbing = false;

    public CapstoneArm(HardwareMap hardwareMap) {
        capExtension = hardwareMap.get(DcMotor.class, "CapExtension");
        capExtension.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        capExtension.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        capExtension.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        grabber = hardwareMap.get(Servo.class, "CapstoneGrabber");
        grabber.setPosition(0.85);

    }

    public void setPower(double power) {
        capExtension.setPower(power);
    }

    public void goToPosition(int armPosition) {
        capExtension.setTargetPosition(armPosition);
        capExtension.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        if(armPosition == 0) {
            capExtension.setPower(0.3);
        } else {
            capExtension.setPower(0.5);
        }
        while (capExtension.isBusy()) {

        }
        capExtension.setPower(0);
        capExtension.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void setGrabberPosition(double position) {
        grabber.setPosition(position);
    }

    public synchronized void toggleGrab() {
        if(isGrabbing) {
            setGrabberPosition(0.85);
            isGrabbing = false;
        } else {
            setGrabberPosition(1);
            isGrabbing = true;
        }
    }

    public double[] getTelemetry() {
        return new double[] {capExtension.getCurrentPosition(), grabber.getPosition()};
    }

}
