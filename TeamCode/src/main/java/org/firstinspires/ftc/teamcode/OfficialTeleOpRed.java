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


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


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

@TeleOp(name="TeleOp Red", group="Linear Opmode")
//@Disabled
public class OfficialTeleOpRed extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    DcMotor carousel;
    DcMotor extender;
    DcMotor magneticArm;
    Servo grabber;


    static final double INCREMENT   = 0.01;     // amount to slew servo each CYCLE_MS cycle
    static final int    CYCLE_MS    =   50;     // period of each cycle
    static final double MAX_POS     =  1.0;     // Maximum rotational position
    static final double MIN_POS     =  0.0;     // Minimum rotational position
    double servoPosition = 0;

    Thread extendingArmThread;
    Thread carouselThread;
    Thread magneticArmThread;

    boolean extenderIsExtended = false;
    boolean magneticIsExtended = false;
    boolean isGrabbing = true;
    boolean servoMoving = false;
    @Override
    public void runOpMode() {
        DriveTrain drive = new DriveTrain(hardwareMap);
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        carousel = hardwareMap.get(DcMotor.class, "Carousel");
        carousel.setDirection(DcMotor.Direction.REVERSE);

        extender = hardwareMap.get(DcMotor.class, "ExtensionArm");
        extender.setDirection(DcMotor.Direction.FORWARD);
        extender.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extender.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);



        grabber = hardwareMap.get(Servo.class, "Grabber");

        grabber.setPosition(servoPosition);

        carouselThread = new Thread(){
            @Override
            public void run(){
                while(opModeIsActive()){
                    carouselUpdate();
                }
            }
        };

        magneticArmThread = new Thread() {
            @Override
            public void run(){
                while(opModeIsActive()){
                    magneticArmUpdate();
                }
            }

        };

        extendingArmThread = new Thread(){
            @Override
            public void run(){
                while(opModeIsActive()){
                    extendingArmUpdate();
                }
            }
        };
        //Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();
        extendingArmThread.start();
        carouselThread.start();
        //extendArm(200);
        grabber.setPosition(servoPosition);
        //extendArm(300);

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            if (gamepad1.right_bumper) {
                drive.setPower(gamepad1.left_stick_y, gamepad1.right_stick_x * .6);
            } else {
                drive.setPower(gamepad1.left_stick_y / 2, gamepad1.right_stick_x / 2);
            }

            if(carousel.isBusy()) {
                drive.setPower(0.1, 0);
            }



            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Extender Tics", "Tics: " + extender.getCurrentPosition());
            telemetry.addData("Servo Position", servoPosition);
            telemetry.addData("Servo Position Actual", grabber.getPosition());
            //telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
            telemetry.update();
        }
    }

    private void carouselUpdate() {

        if(gamepad1.dpad_right) {
            carousel.setPower(0.7);
        } else if(gamepad1.dpad_left) {
            carousel.setPower(-0.7);
        } else {
            carousel.setPower(0);
        }

        if (gamepad1.a) {
            spinCarousel(-4000);
        }

        if (gamepad1.y) {
            grab(0.5);
        }

    }

    private void magneticArmUpdate(){
        /*
        if (gamepad1.b && !magneticIsExtended) {
            magneticIsExtended = true;
            magneticExtend(5000);
        }

        if (gamepad1.x && magneticIsExtended) {
            magneticIsExtended = false;
            magneticExtend(0);
        }

         */

    }

    private void extendingArmUpdate() {
        if (gamepad1.left_trigger > 0.1 && extenderIsExtended) {
            extenderIsExtended = false;
            extendArm(300);
        } else if (gamepad1.right_trigger > 0.1 && !extenderIsExtended) {
            extenderIsExtended = true;
            extendArm(8900);
        }

        if(gamepad1.dpad_up) {
            extender.setPower(0.5);
        } else if(gamepad1.dpad_down) {
            extender.setPower(-0.3);
        } else {
            extender.setPower(0);
        }

        if(gamepad1.b) {
            extendArm(6295);
        }



        // ~6.28 inches per rotation, need to extend 28 inches, Don't Overshoot!!!,
        // 4.45 rotations, ~2220.4 tics/rotation
        /*if (gamepad1.b && !isExtended) {
            isExtended = true;
            extendArm(8900);
        }

        if (gamepad1.x && isExtended) {
            isExtended = false;
            extendArm(300);
        }

         */

        if (gamepad1.left_bumper) {
            extender.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            extender.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

    }


    boolean previousYState;
    public synchronized void grab(double position){
        double targetPosition;
        servoMoving = true;

        previousYState = gamepad1.y;

        if(isGrabbing) {
            targetPosition = 0;
        }else{
            targetPosition = position;
        }

        servoPosition = targetPosition;
        grabber.setPosition(servoPosition);

        while(previousYState == gamepad1.y){

        }
        isGrabbing = !isGrabbing;

    }

    public void spinCarousel(int tics) {



        carousel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        carousel.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        carousel.setTargetPosition(tics);

        double power;

        carousel.setPower(0.5);

        while (carousel.isBusy()) {
            power = 2 * (1- (Math.abs(carousel.getCurrentPosition() - carousel.getTargetPosition()) / 4000.0));
            if (power < 0.5) {
                carousel.setPower(0.5);
            } else if (power > 1){
                carousel.setPower(1);
            } else {
                carousel.setPower(power);
            }
        }

        carousel.setPower(0);

        carousel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }

    public void extendArm(int armPosition) {

        extender.setTargetPosition(armPosition);

        extender.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        if(!extenderIsExtended) {
            extender.setPower(0.75);
        } else {
            extender.setPower(1);
        }

        while (extender.isBusy()) {

        }

        extender.setPower(0);

        extender.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void magneticExtend(int armPosition){
        magneticArm.setTargetPosition(armPosition);

        magneticArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        magneticArm.setPower(0.5);

        while(magneticArm.isBusy()){

        }

        magneticArm.setPower(0);

        magneticArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


    }

}

