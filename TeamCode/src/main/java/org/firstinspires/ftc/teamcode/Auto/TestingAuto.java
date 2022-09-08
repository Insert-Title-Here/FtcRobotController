package org.firstinspires.ftc.teamcode.Auto;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Intake;
import org.firstinspires.ftc.teamcode.MecDrive;
import org.firstinspires.ftc.teamcode.MecanumDriveTrain;
import org.firstinspires.ftc.teamcode.OpModeWrapper;
import org.firstinspires.ftc.teamcode.Point;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.Testing.RobotK;

import java.io.FileNotFoundException;

@Autonomous(name = "StuffAuto")
public class TestingAuto extends OpModeWrapper {

    Robot robot;
    MecDrive drive;


    @Override
    protected void onInitialize() throws FileNotFoundException {
        robot = new Robot(hardwareMap);
        drive = new MecDrive(hardwareMap, robot, false, telemetry);

    }

    @Override
    protected void onStart() {

        robot.start();
        drive.moveToPosition(new Point(0, 500), 10);

        while(opModeIsActive()){
/*
            telemetry.addData("fl ", drive.fl.getCurrentPosition());
            telemetry.addData("fr ", drive.fr.getCurrentPosition());
            telemetry.addData("bl ", drive.bl.getCurrentPosition());
            telemetry.addData("br ", drive.br.getCurrentPosition());

            telemetry.addData("fl ", robot.drive.fl.getCurrentPosition());
            telemetry.addData("fr ", robot.drive.fr.getCurrentPosition());
            telemetry.addData("bl ", robot.drive.bl.getCurrentPosition());
            telemetry.addData("br ", robot.drive.br.getCurrentPosition());

            telemetry.update();
*/
        }
    }


    @Override
    protected void onStop() {

    }
}
