package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.List;

public class DriveTrain {

    DcMotor lf;
    DcMotor rf;
    DcMotor lb;
    DcMotor rb;

    DcMotor[] motors;

    public DriveTrain(HardwareMap hardwareMap) {
        lf  = hardwareMap.get(DcMotor.class, "LeftFrontDrive");
        rf = hardwareMap.get(DcMotor.class, "RightFrontDrive");
        lb = hardwareMap.get(DcMotor.class, "LeftBackDrive");
        rb = hardwareMap.get(DcMotor.class, "RightBackDrive");

        motors = new DcMotor[]{lf, rf, lb, rb};

        lf.setDirection(Direction.FORWARD);
        rf.setDirection(Direction.REVERSE);
        lb.setDirection(Direction.FORWARD);
        rb.setDirection(Direction.REVERSE);

        for(DcMotor motor : motors) {
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
    }

    public void brake() {
        lf.setPower(0);
        rf.setPower(0);
        lb.setPower(0);
        rb.setPower(0);
    }

    public void linear (double speed) {
        lf.setPower(speed);
        rf.setPower(speed);
        lb.setPower(speed);
        rb.setPower(speed);
    }

    //+ = clockwise, - = counterclockwise
    public void rotate (double speed) {
        lf.setPower(speed);
        rf.setPower(-speed);
        lb.setPower(speed);
        rb.setPower(-speed);
    }

    public void setPower (double linear, double rotational) {
        lf.setPower(linear + rotational);
        rf.setPower(linear - rotational);
        lb.setPower(linear + rotational);
        rb.setPower(linear - rotational);
    }

    public int[] getEncoders() {
        return new int[]{lf.getCurrentPosition(), rf.getCurrentPosition(),
                lb.getCurrentPosition(), rb.getCurrentPosition()};

    }

}
