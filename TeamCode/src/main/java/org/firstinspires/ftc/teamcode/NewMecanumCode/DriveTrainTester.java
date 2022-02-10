package org.firstinspires.ftc.teamcode.NewMecanumCode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
public class DriveTrainTester {



    DcMotor backLeft;
    DcMotor backRight;
    DcMotor frontLeft;
    DcMotor frontRight;

    public DriveTrainTester(HardwareMap hardwareMap){
        frontLeft= hardwareMap.dcMotor.get("FrontLeftDrive");
        frontRight = hardwareMap.dcMotor.get("FrontRightDrive");
        backLeft = hardwareMap.dcMotor.get("BackLeftDrive");
        backRight = hardwareMap.dcMotor.get("BackRightDrive");

        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }

    public void move(double power) {
        frontRight.setPower(power);
        frontLeft.setPower(power);
        backRight.setPower(power);
        backLeft.setPower(power);
    }


    public void stopMoving(){
        frontRight.setPower(0);
        frontLeft.setPower(0);
        backRight.setPower(0);
        backLeft.setPower(0);
    }

    public void rotate(double power){
        frontRight.setPower(power);
        frontLeft.setPower(-power);
        backRight.setPower(power);
        backLeft.setPower(-power);
    }

    public void setPower(double power, double rotation){
        frontRight.setPower(power + rotation);
        frontLeft.setPower(power - rotation);
        backRight.setPower(power + rotation);
        backLeft.setPower(power - rotation);
    }

    public void strafe( double direction){
        frontRight.setPower(-direction);
        frontLeft.setPower(direction);
        backRight.setPower(direction);
        backLeft.setPower(-direction);
    }

    public void driveEncoders(int tics, double power){
        telemetry.addData("fl:", frontLeft.getCurrentPosition());
        telemetry.addData("bl:", backLeft.getCurrentPosition());
        telemetry.addData("fr:", frontRight.getCurrentPosition());
        telemetry.addData("br:", backRight.getCurrentPosition());
        telemetry.addData("isFar", isFar(tics));
        while(isFar(tics) > 10){
            setPower(power,0);

            telemetry.update();
        }
        setPower(0,0);



    }

    public double isFar(int tics){

        return((tics - frontLeft.getCurrentPosition()) +(tics - frontRight.getCurrentPosition()) +
                (tics - backLeft.getCurrentPosition()) +(tics - backRight.getCurrentPosition()))/4;

    }
}


