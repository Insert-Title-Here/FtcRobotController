package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

//This is a experiment to see if i can acutally make our robot move
//by DragonStryke

@Autonomous(name = "TestAutoOpModeC", group = "LinearOpmode")
@Disabled

public class testAutoOpModeC extends LinearOpMode {
    //opmode members

    DriveTrain drivetrain = new DriveTrain();
    private DcMotor carousel;


    @Override
    public void runOpmode() {
        //telemetry data to signal robot waiting
        telemetry.addData("Status", "Ready to run");    //
        telemetry.update();

        //start moving
        drivetrain.setPower(0.5, 0);
        Thread.sleep(1000);
        drivetrain.setPower(0, 0.5);
        Thread.sleep(500);
        drivetrain.setPower(0.5, 0);
        Thread.sleep(1000);
        drivetrain.brake();

    }
}
