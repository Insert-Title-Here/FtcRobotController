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

    double power;

    // Makes it easier to remember which is which
    int LINEAR = 0;
    int ROTATION = 1;



    public DriveTrain(HardwareMap hardwareMap) {
        lf  = hardwareMap.get(DcMotor.class, "LeftFrontDrive");
        rf = hardwareMap.get(DcMotor.class, "RightFrontDrive");
        //lb = hardwareMap.get(DcMotor.class, "LeftBackDrive");
        //rb = hardwareMap.get(DcMotor.class, "RightBackDrive");

        motors = new DcMotor[]{lf, rf}; //motors = new DcMotor[]{lf, rf, lb, rb};

        lf.setDirection(Direction.FORWARD);
        rf.setDirection(Direction.REVERSE); //keep in mind that this also reverses direction of encoders
        //lb.setDirection(Direction.FORWARD);
        //rb.setDirection(Direction.REVERSE);
/*
        for(DcMotor motor : motors) {
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
*/
    }

    public void brake() {
        lf.setPower(0);
        rf.setPower(0);
        //lb.setPower(0);
        //rb.setPower(0);
    }

    public void linear (double speed) {
        lf.setPower(speed);
        rf.setPower(speed);
        //lb.setPower(speed);
        //rb.setPower(speed);
    }

    //+ = clockwise, - = counterclockwise
    public void rotate (double speed) {
        lf.setPower(speed);
        rf.setPower(-speed);
        //lb.setPower(speed);
        //rb.setPower(-speed);
    }

    public void setPower (double linear, double rotational) {
        lf.setPower(linear - rotational);
        rf.setPower(linear + rotational);
        //lb.setPower(linear - rotational);
        //rb.setPower(linear + rotational);
    }

    public void goToPosition (int tics, int mode) {
        for(DcMotor motor : motors) {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setTargetPosition(tics);
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

        if(mode == 1) {
            rf.setDirection(Direction.REVERSE);
            rb.setDirection(Direction.REVERSE);
        }

        setPower(1, 0);

        while (lf.isBusy()) {
            power = Math.abs(lf.getCurrentPosition() - tics) / (double) tics;
            if (power < 0.1) {
                setPower(0.3, 0);
            } else if (power > 1) {
                setPower(1, 0);
            } else {
                setPower(power, 0);
            }
        }

        brake();

        for(DcMotor motor : motors) {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        if(mode == 1) {
            rf.setDirection(Direction.FORWARD);
            //rb.setDirection(Direction.FORWARD);
        }
    }

    //probably want to use a bulk read for this, set up a sophisticated control loop,
    //for now its ok but a later season thing maybe, feel free to ask me about it 
    public int[] getEncoders() {
        return new int[]{lf.getCurrentPosition(), rf.getCurrentPosition()};
        //return new int[]{lf.getCurrentPosition(), rf.getCurrentPosition(),
        //                lb.getCurrentPosition(), rb.getCurrentPosition()};

    }

}
