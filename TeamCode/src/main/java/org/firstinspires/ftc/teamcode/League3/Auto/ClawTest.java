package org.firstinspires.ftc.teamcode.League3.Auto;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.League3.Common.ScoringSystem;

//use servo programmer to find out 0 and 1 position

//@Autonomous
public class ClawTest extends LinearOpMode {
    ScoringSystem intake;

    @Override
    public void runOpMode() throws InterruptedException {
        intake = new ScoringSystem(hardwareMap, telemetry);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        //use servo programmer to find out 0 and 1 position/set those positions to values w/ Mr. Swartz
        intake.setClawPosition(0.5); //0.5 is currently an arbitrary position
        sleep(1000);
        intake.setClawPosition(0);
        sleep(500);

        while(opModeIsActive()) {
            telemetry.addData("servoPosition", intake.getClawPosition()); // expected pos.
            telemetry.update();
        }
    }
}
