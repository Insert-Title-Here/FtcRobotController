package org.firstinspires.ftc.teamcode.Competition.DebuggingOrTuning;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Competition.Interleagues.Common.Vector2D;
import org.firstinspires.ftc.teamcode.Competition.Interleagues.Subsystems.Used.MecDrive;


//TODO: figure out bulk read

@Disabled
@TeleOp
public class DriveTest extends LinearOpMode {

    private final double NORMAL_LINEAR_MODIFIER = 0.8;
    private final double NORMAL_ROTATIONAL_MODIFIER = 0.8;
    private final double SPRINT_LINEAR_MODIFIER = 1;
    private final double SPRINT_ROTATIONAL_MODIFIER = 1;


    MecDrive drive;

    @Override
    public void runOpMode() throws InterruptedException {
        drive = new MecDrive(hardwareMap, false, telemetry);



        waitForStart();

        while(opModeIsActive()){

            if (gamepad1.right_bumper) {
            drive.setPower(new Vector2D(gamepad1.left_stick_x * SPRINT_LINEAR_MODIFIER, gamepad1.left_stick_y * SPRINT_LINEAR_MODIFIER), gamepad1.right_stick_x * SPRINT_ROTATIONAL_MODIFIER, false);
            } else {
            drive.setPower(new Vector2D(gamepad1.left_stick_x * NORMAL_LINEAR_MODIFIER, gamepad1.left_stick_y * NORMAL_LINEAR_MODIFIER), gamepad1.right_stick_x * NORMAL_ROTATIONAL_MODIFIER, false);
            }




            telemetry.addData("flPos: ", drive.getFLEncoder());
            telemetry.addData("frPos: ", drive.getFREncoder());
            telemetry.addData("blPos: ", drive.getBLEncoder());
            telemetry.addData("brPos: ", drive.getBREncoder());
            telemetry.update();

        }
    }
}
