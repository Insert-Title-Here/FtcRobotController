package org.firstinspires.ftc.teamcode.NewMecanumCode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "backforth", group = "Linear Opmode")
@Disabled
public class BaseDrive extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();

    DcMotor backLeft;
    DcMotor backRight;
    DcMotor frontLeft;
    DcMotor frontRight;

    double power = 0.5;

    @Override
    private void runOpMode() throws InterruptedException {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        frontLeft= hardwareMap.dcMotor.get("FrontLeftDrive");
        frontRight = hardwareMap.dcMotor.get("FrontRightDrive");
        backLeft = hardwareMap.dcMotor.get("BackLeftDrive");
        backRight = hardwareMap.dcMotor.get("BackRightDrive");

        waitForStart();
        runtime.reset();

        frontLeft.setPower(power);
        frontRight.setPower(power);
        backLeft.setPower(power);
        backRight.setPower(power);

        sleep(2000);

        power = -0.5;

        frontLeft.setPower(power);
        frontRight.setPower(power);
        backLeft.setPower(power);
        backRight.setPower(power);

    }

}
