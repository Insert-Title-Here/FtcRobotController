package org.firstinspires.ftc.teamcode.Testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Common.MecDriveV2;

@TeleOp
public class TestingPDFThing extends LinearOpMode {

    static final double target = 0, kp = 0, kd = 0, flF = 0, frF = 0, blF = 0, brF = 0;
    MecDriveV2 drive;

    @Override
    public void runOpMode() throws InterruptedException {

        drive = new MecDriveV2(hardwareMap, false, telemetry, new ElapsedTime());

        waitForStart();

        while (opModeIsActive()) {
            drive.drivePDFTesting(target, flF, frF, blF, brF, kp, kp);
        }

        drive.simpleBrake();

    }


}
