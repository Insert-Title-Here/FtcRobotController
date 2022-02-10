package org.firstinspires.ftc.teamcode.NewMecanumCode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "Test TeleOp Thing", group = "Linear Opmode")
//@Disabled
public class TestTeleOp extends LinearOpMode  {


    private ElapsedTime runtime = new ElapsedTime();

    double power = 0.5;

    @Override
    public void runOpMode() throws InterruptedException {
        DriveTrainTester myDriveTrain =  new DriveTrainTester(hardwareMap);


        waitForStart();


        while(opModeIsActive()){
            myDriveTrain.setPower(gamepad1.left_stick_y, gamepad1.right_stick_x);
        }

    }

}


