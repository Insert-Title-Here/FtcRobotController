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

@Autonomous(name = "Moving Auto")
public class TestingAuto extends OpModeWrapper {

    Robot robot;
    MecDrive drive;


    @Override
    protected void onInitialize() throws FileNotFoundException {
        robot = new Robot(hardwareMap);
        drive = new MecDrive(hardwareMap, robot, false, telemetry);

        robot.start();


    }

    @Override
    protected void onStart() {


        drive.newMoveToPosition(new Point(1000, 1000), 0.5);

        /*while(opModeIsActive()){
            LynxModule.BulkData data = robot.getBulkPacket(true);

            telemetry.addData("fl ", data.getMotorCurrentPosition(0));
            telemetry.addData("fr ", data.getMotorCurrentPosition(1));
            telemetry.addData("bl ", data.getMotorCurrentPosition(2));
            telemetry.addData("br ", data.getMotorCurrentPosition(3));


            telemetry.update();

        }*/
    }


    @Override
    protected void onStop() {

    }
}
