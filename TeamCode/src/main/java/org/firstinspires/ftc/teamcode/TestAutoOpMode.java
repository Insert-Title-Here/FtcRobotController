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

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;


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

@Autonomous(name="TestAutoOpMode", group="Linear Opmode")
//@Disabled
public class TestAutoOpMode extends LinearOpMode {

    BNO055IMU imu;
    DcMotor extender;
    Servo grabber;

    double servoPosition = 0.3;

    boolean isExtended = false;
    boolean isGrabbing = true;
    boolean servoMoving = false;
    boolean previousYState;
    Thread armThread;

    Orientation angles;
    Acceleration gravity;

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {

        DriveTrain drive = new DriveTrain(hardwareMap);
        extender = hardwareMap.get(DcMotor.class, "ExtensionArm");
        extender.setDirection(DcMotor.Direction.FORWARD);
        extender.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extender.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        grabber = hardwareMap.get(Servo.class, "Grabber");

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);


        waitForStart();


        drive.goToPosition(-200, false, 0.3);
        extendArm(500);


        while(!(imu.getAngularOrientation().firstAngle < -90)){
            telemetry.addData("angle", imu.getAngularOrientation().firstAngle);
            telemetry.update();
            drive.lf.setPower(-0.8);
            drive.rf.setPower(0.8);
        }

        telemetry.addData("Angle","Degree: " + imu.getAngularOrientation().firstAngle);

        drive.goToPosition(-1500, false, 0.5);

        /*
        while(opModeIsActive()) {

            telemetry.addData("Angle", "Something: " + imu.getAngularOrientation().firstAngle);
            telemetry.addData("Encoder", drive.lf.getCurrentPosition());


            telemetry.update();
        }

         */
        /*telemetry.addData("Status", "Initialized");
        telemetry.update();

        DriveTrain drive = new DriveTrain(hardwareMap);

        double power;
        int targetTics = 435;

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Store the number of tics the motors have rotated into a variable
            int leftTics = drive.lf.getCurrentPosition();
            int rightTics = drive.rf.getCurrentPosition();

            // 434.7 tics is about 1 foot
            // 537.7 tics per revolution
            // If the robot has travelled farther than a foot, stop it
            power = 1.0 - (1.0 / (targetTics - leftTics));
            if (leftTics > targetTics) {
                drive.brake();

            } else {
                drive.setPower(.5, 0);
            }

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            //telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower)
            telemetry.addData("Left Tics", "Tics: " + leftTics);
            telemetry.addData("Right Tics", "Tics: " + rightTics);
            telemetry.addData("Power", "Power: " + power);
            telemetry.update();

         */

    }

    public void extendArm(int armPosition) {

        extender.setTargetPosition(armPosition);

        extender.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        extender.setPower(0.5);

        while (extender.isBusy()) {

        }

        extender.setPower(0);

        extender.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

}
