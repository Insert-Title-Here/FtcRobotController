package org.firstinspires.ftc.teamcode.NewMecanumCode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "Underscore", group = "Linear Opmode")
//@Disabled
public class BaseDriveSolomon extends LinearOpMode  {


    private ElapsedTime runtime = new ElapsedTime();

    DcMotor backLeft;
    DcMotor backRight;
    DcMotor frontLeft;
    DcMotor frontRight;

    double power = 0.5;

    @Override
    public void runOpMode() throws InterruptedException {
        DriveTrainTester myDriveTrain =  new DriveTrainTester(hardwareMap);


        waitForStart();

        myDriveTrain.strafe(.5);
        sleep(1000);
        myDriveTrain.stopMoving();

        myDriveTrain.strafe(-.5);
        sleep(1000);
        myDriveTrain.stopMoving();

        myDriveTrain.driveEncoders(1400,.5);

/*

       myDriveTrain.move(.5);

       sleep(1000);

       myDriveTrain.stopMoving();

       myDriveTrain.rotate(1);

       sleep(1000);

       myDriveTrain.stopMoving();


*/

    }

}


