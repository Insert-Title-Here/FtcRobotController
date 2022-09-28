package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Common.MecanumDrive;
import org.firstinspires.ftc.teamcode.Common.TankDrive;
import org.firstinspires.ftc.teamcode.Common.Vector2D;

@Disabled
@TeleOp
public class TankTeleOp extends LinearOpMode {

    TankDrive drive;


    @Override
    public void runOpMode() throws InterruptedException {

        drive = new TankDrive(hardwareMap);


        waitForStart();

        while(opModeIsActive()){
            drive.setPower(gamepad1.left_stick_x);
        }
    }
}