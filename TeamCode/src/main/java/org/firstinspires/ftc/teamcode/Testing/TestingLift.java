package org.firstinspires.ftc.teamcode.Testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Testing.Command.Talons.TalonsScoringSystem;

@TeleOp
public class TestingLift extends LinearOpMode {

    TalonsScoringSystem lift;


    @Override
    public void runOpMode() throws InterruptedException {

        lift = new TalonsScoringSystem(hardwareMap);

        waitForStart();

        while(opModeIsActive() && !isStopRequested()){
            if(gamepad1.a){
                lift.extend(0.5, gamepad1);
            }

            if(gamepad1.b){
                lift.retract(0.5, gamepad1);

            }

            if(gamepad1.y){
                stop();
            }
        }
    }
}
