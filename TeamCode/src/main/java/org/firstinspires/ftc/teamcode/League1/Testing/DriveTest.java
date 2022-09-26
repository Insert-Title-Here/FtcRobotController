package org.firstinspires.ftc.teamcode.League1.Testing;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.League1.Common.Constants;
import org.firstinspires.ftc.teamcode.League1.Common.Robot;
import org.firstinspires.ftc.teamcode.League1.Common.Vector2D;
import org.firstinspires.ftc.teamcode.League1.Subsystems.MecDrive;
import org.firstinspires.ftc.teamcode.League1.Subsystems.ScoringSystem;


//TODO: figure out bulk read

@TeleOp
public class DriveTest extends LinearOpMode {

    private final double NORMAL_LINEAR_MODIFIER = 0.8;
    private final double NORMAL_ROTATIONAL_MODIFIER = 0.8;
    private final double SPRINT_LINEAR_MODIFIER = 1;
    private final double SPRINT_ROTATIONAL_MODIFIER = 1;


    MecDrive drive;
    Robot robot;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(hardwareMap);
        drive = new MecDrive(hardwareMap, robot, false, telemetry);

        robot.start();


        waitForStart();

        while(opModeIsActive()){

            if (gamepad1.right_bumper) {
            drive.setPower(new Vector2D(gamepad1.left_stick_x * SPRINT_LINEAR_MODIFIER, gamepad1.left_stick_y * SPRINT_LINEAR_MODIFIER), gamepad1.right_stick_x * SPRINT_ROTATIONAL_MODIFIER, false);
            } else {
            drive.setPower(new Vector2D(gamepad1.left_stick_x * NORMAL_LINEAR_MODIFIER, gamepad1.left_stick_y * NORMAL_LINEAR_MODIFIER), gamepad1.right_stick_x * NORMAL_ROTATIONAL_MODIFIER, false);
            }

            LynxModule.BulkData data = robot.getBulkPacket(true);



            telemetry.addData("flPos: ", data.getMotorCurrentPosition(0));
            telemetry.addData("frPos: ", data.getMotorCurrentPosition(1));
            telemetry.addData("blPos: ", data.getMotorCurrentPosition(2));
            telemetry.addData("brPos: ", data.getMotorCurrentPosition(3));
            telemetry.update();

        }
    }
}
