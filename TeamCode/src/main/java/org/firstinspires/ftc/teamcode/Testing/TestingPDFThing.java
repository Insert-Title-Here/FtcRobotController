package org.firstinspires.ftc.teamcode.Testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Common.MecDriveV2;

@TeleOp
@Config
public class TestingPDFThing extends LinearOpMode {

    public static double target = 0, kp = 0, kd = 0, flF = 0, frF = 0, blF = 0, brF = 0;
    MecDriveV2 drive;

    @Override
    public void runOpMode() throws InterruptedException {

        drive = new MecDriveV2(hardwareMap, false, telemetry, new ElapsedTime());
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        waitForStart();

        while (opModeIsActive()) {
            //drive.drivePDFTesting(target, flF, frF, blF, brF, kp, kp);
            drive.setFLVelocity(flF);

            telemetry.addData("Target:", target);
            telemetry.addData("flPos:", drive.getFLEncoder());
            telemetry.addData("frPos:", drive.getFREncoder());
            telemetry.addData("blPos:", drive.getBLEncoder());
            telemetry.addData("brPos:", drive.getBREncoder());
            telemetry.update();

        }

        drive.simpleBrake();

    }


}
