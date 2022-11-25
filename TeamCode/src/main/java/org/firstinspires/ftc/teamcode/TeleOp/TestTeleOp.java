package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Common.MecanumDrive;
import org.firstinspires.ftc.teamcode.Common.ScoringSystem;

import java.util.concurrent.atomic.AtomicBoolean;

@TeleOp
public class TestTeleOp extends LinearOpMode {
    //TODO: change names if you want to
    MecanumDrive drive;
    ScoringSystem score;
    AtomicBoolean discontinue;

    private final double NORMAL_LINEAR_MODIFIER = 0.55;
    private final double NORMAL_ROTATIONAL_MODIFIER = 0.6;
    private final double SPRINT_LINEAR_MODIFIER = 1;
    private final double SPRINT_ROTATIONAL_MODIFIER = 1;
    @Override
    public void runOpMode() throws InterruptedException {
        drive = new MecanumDrive(hardwareMap, telemetry);
        score = new ScoringSystem(hardwareMap, telemetry);
        discontinue = new AtomicBoolean();
        discontinue.set(false);
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        //Open
        //Thread for the slides


        score.setClawPosition(0);
        waitForStart();

        while(opModeIsActive()){
            //Limits robot movement from controls to only the 4 cardinal directions N,S,W,E
            if (score.getClawPosition() == 0.0) {
                try {
                    score.grabCone(true);
                    discontinue.set(false);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
            }

        }
        drive.setPower(0, 0, 0, 0);
        score.setPower(0);
        score.setClawPosition(1);
    }
}