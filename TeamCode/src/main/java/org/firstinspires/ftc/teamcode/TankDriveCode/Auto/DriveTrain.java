package org.firstinspires.ftc.teamcode.TankDriveCode.Auto;

import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.lynx.LynxEmbeddedIMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.util.List;

public class DriveTrain {

    DcMotor lf;
    DcMotor rf;
    DcMotor lb;
    DcMotor rb;
    BNO055IMU imu;
    Orientation angles;

    DcMotor[] motors;

    double power;

    int rampUpSpeed = 500;

    // Makes it easier to remember which is which
    boolean LINEAR = false;
    boolean ROTATION = true;

    public double checkOrientation(){
        angles = this.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return angles.firstAngle;
    }

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

        for(DcMotor motor : motors) {
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }


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

    public void goToPosition (int tics, boolean rotate, double power) {
        for(DcMotor motor: motors){
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setTargetPosition(tics);
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
/*
        if(rotate) {
            if(tics < 0) {
                setPower(0, -0.3);
            } else {
                setPower(0, 0.3);
            }
        } else {
            if(tics < 0) {
                setPower(-0.3, 0);
            } else {
                setPower(0.3, 0);
            }
        }
*/
        setPower(power, 0);
        while (lf.isBusy()) {

        }

        brake();

        //for(DcMotor motor : motors) {
        //    motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //    motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //}
    }

    public void rotateToPosition(double distance) {

    }

    //probably want to use a bulk read for this, set up a sophisticated control loop,
    //for now its ok but a later season thing maybe, feel free to ask me about it 
    public int[] getEncoders() {
        return new int[]{lf.getCurrentPosition(), rf.getCurrentPosition()};
        //return new int[]{lf.getCurrentPosition(), rf.getCurrentPosition(),
        //                lb.getCurrentPosition(), rb.getCurrentPosition()};

    }

}
