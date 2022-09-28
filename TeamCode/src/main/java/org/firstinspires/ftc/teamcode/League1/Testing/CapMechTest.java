package org.firstinspires.ftc.teamcode.League1.Testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.teamcode.League1.Common.Constants;
import org.firstinspires.ftc.teamcode.League1.Common.Robot;
import org.firstinspires.ftc.teamcode.League1.Common.Vector2D;
import org.firstinspires.ftc.teamcode.League1.Subsystems.EndgameSystems;
import org.firstinspires.ftc.teamcode.League1.Subsystems.MecDrive;
import org.firstinspires.ftc.teamcode.League1.Subsystems.ScoringSystem;


//TODO: figure out bulk read

@TeleOp
public class CapMechTest extends LinearOpMode {


    EndgameSystems systems;
    boolean previousLeft, previousRight, previousUp, previousDown;

    @Override
    public void runOpMode() throws InterruptedException {

        systems = new EndgameSystems(hardwareMap);
        waitForStart();

        while(opModeIsActive()){

            /*if (gamepad1.right_trigger > 0.1) {
                servo.setPower(1);
            } else if(gamepad1.left_trigger > 0.1) {
                servo.setPower(-1);
            }else{
                servo.setPower(0);
            }

             */


            if(gamepad2.right_trigger > 0 || gamepad2.left_trigger > 0) {
                double val = gamepad2.right_trigger - gamepad2.left_trigger;
                systems.setCapstoneExtensionPower(val);
            } else if (gamepad2.right_bumper) {
                systems.setCapstoneExtensionPower(0.5);
            } else if (gamepad2.left_bumper) {
                systems.setCapstoneExtensionPower(-0.5);
            } else{
                systems.setCapstoneExtensionPower(0);
            }


            telemetry.addData("servo pos", systems.getYCapPosition());
            telemetry.update();


            double yPos = systems.getYCapPosition();
            systems.setXCapstoneRotatePower(gamepad2.left_stick_x);

//        telemetry.addData("rsy", gamepad2.right_stick_y);
//        telemetry.update();
            systems.setYCapPosition(yPos - systems.map(gamepad2.right_stick_y, -1, 1, -0.0010, 0.0010));

            if (gamepad2.x) {
                systems.zeroCap();
            } else if (gamepad2.dpad_right && previousRight != gamepad2.dpad_right) {
                systems.setXCapSpeedDivisor(7);
            } else if (gamepad2.dpad_left && previousLeft != gamepad2.dpad_left) {
                systems.setXCapSpeedDivisor(4);
            }

            previousLeft = gamepad2.dpad_left;
            previousRight = gamepad2.dpad_right;
            previousUp = gamepad2.dpad_up;
            previousDown = gamepad2.dpad_down;
        }

        systems.zeroCap();
        systems.setXCapstoneRotatePower(0);
        systems.setCapstoneExtensionPower(0);

    }
}
