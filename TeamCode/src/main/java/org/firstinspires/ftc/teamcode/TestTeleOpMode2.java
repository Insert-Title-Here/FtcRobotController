/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import static java.lang.Math.round;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="Test TeleOp Mode 2", group="Linear Opmode")
//@Disabled
public class TestTeleOpMode2 extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    DcMotor carousel;
    DcMotor extender;
    Servo grabber;

    Thread armThread;

    boolean isExtended = false;
    boolean isGrabbing = false;
    @Override
    public void runOpMode() {
        DriveTrain drive = new DriveTrain(hardwareMap);
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        carousel = hardwareMap.get(DcMotor.class, "Carousel");
        carousel.setDirection(DcMotor.Direction.REVERSE);

        extender = hardwareMap.get(DcMotor.class, "ExtensionArm");
        extender.setDirection(DcMotor.Direction.FORWARD);

        grabber = hardwareMap.get(Servo.class, "Grabber");

        armThread = new Thread(){
            @Override
            public void run(){
                while(opModeIsActive()){
                    armUpdate();
                }
            }
        };
        //Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();
        armThread.start();
        //extendArm(200);
        grabber.setPosition(0.5);

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            if (gamepad1.right_bumper) {
                drive.setPower(gamepad1.left_stick_y, gamepad1.right_stick_x);
            } else {
                drive.setPower(gamepad1.left_stick_y / 2, gamepad1.right_stick_x / 2);
            }



            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Extender Tics", "Tics: " + extender.getCurrentPosition());
            //telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
            telemetry.update();
        }
    }

    private void armUpdate() {
        if (gamepad1.left_trigger > 0.1) {
            extender.setPower(gamepad1.left_trigger);
        } else if (gamepad1.right_trigger > 0.1) {
            extender.setPower(-gamepad1.right_trigger);
        } else {
            extender.setPower(0);
        }

        if (gamepad1.a) {
            spinCarousel(4000);
        }

        // ~6.28 inches per rotation, need to extend 28 inches, Don't Overshoot!!!,
        // 4.45 rotations, ~2220.4 tics/rotation
        if (gamepad1.b && !isExtended) {
            isExtended = true;
            extendArm(2000);
        }

        if (gamepad1.x && isExtended) {
            isExtended = false;
            extendArm(-2000);
        }

        if (gamepad1.left_bumper) {
            extendArm(-200);
        }
        if (gamepad1.y) {
            if(isGrabbing){
                grabber.setPosition(1);
                isGrabbing = false;

            }else{
                grabber.setPosition(0.5);
                isGrabbing = true;
            }
        }
    }

    public void spinCarousel(int tics) {



        carousel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        carousel.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        carousel.setTargetPosition(tics);

        double power;

        carousel.setPower(0.1);

        while (carousel.isBusy()) {
            power = 2 * (1- (Math.abs(carousel.getCurrentPosition() - carousel.getTargetPosition()) / 4000.0));
            if (power < 0.1) {
                carousel.setPower(0.3);
            } else if (power > 1){
                carousel.setPower(1);
            } else {
                carousel.setPower(power);
            }
        }

        carousel.setPower(0);

        carousel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }

    public void extendArm(int tics) {
        extender.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        extender.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        extender.setTargetPosition(tics);

        extender.setPower(1);

        while (extender.isBusy()) {

        }

        extender.setPower(0);

        extender.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

}

