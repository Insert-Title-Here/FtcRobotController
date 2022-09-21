package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

//use servo programmer to find out 0 and 1 position

@TeleOp
public class Claw extends LinearOpMode {
    ScoringSystem intake;

    @Override
    public void runOpMode() throws InterruptedException {
        intake = new ScoringSystem(hardwareMap);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        //use servo programmer to find out 0 and 1 position/set those positions to values w/ Mr. Swartz
        intake.setClawPosition(0.5); //0.5 is currently an arbitrary position
        sleep(1000);
        intake.setClawPosition(0);
        sleep(500);

        while(opModeIsActive()) {
            telemetry.addData("servoPosition", intake.claw.getPosition());
            telemetry.update();
        }
    }
}
