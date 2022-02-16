package org.firstinspires.ftc.teamcode.NewMecanumCode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.checkerframework.checker.units.qual.C;

@Autonomous(name = "wheeeeeeeeeee", group = "Linear Opmode")
//@Disabled
public class BaseDrive extends LinearOpMode  {


    private ElapsedTime runtime = new ElapsedTime();

    DcMotor backLeft;
    DcMotor backRight;
    DcMotor frontLeft;
    DcMotor frontRight;

    DcMotor carousel;

    double power = 0.5;

    @Override
    public void runOpMode() throws InterruptedException {
        DriveTrainTester driveTrain =  new DriveTrainTester(hardwareMap);
        CarouselTest carousel = new CarouselTest(hardwareMap);

        waitForStart();

        driveTrain.move(1);
        sleep(1000);
        driveTrain.stopMoving();
        carousel.spin(1);
        sleep(1000);
        carousel.stop();
        driveTrain.strafe(1);
        sleep(500);
        driveTrain.stopMoving();
        sleep(100);
        driveTrain.move(-1);
        sleep(1000);
        driveTrain.stopMoving();
        sleep(100);
        driveTrain.strafe(-1);
        sleep(500);
        driveTrain.stopMoving();
  
    }

}


