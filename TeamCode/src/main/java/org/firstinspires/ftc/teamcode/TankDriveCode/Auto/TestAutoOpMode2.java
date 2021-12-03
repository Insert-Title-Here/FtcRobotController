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

package org.firstinspires.ftc.teamcode.TankDriveCode.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Common.BarcodePipeline;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;


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

@Autonomous(name="TestAutoOpMode2", group="Linear Opmode")
@Disabled
public class TestAutoOpMode2 extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    Thread driveThread;
    DriveTrain drive;
    
    WebcamName wc;
    OpenCvCamera camera;

    BarcodePipeline.BarcodePosition capstonePos;


    // global obj
    static final BarcodePipeline brp = new BarcodePipeline();


    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        drive = new DriveTrain(hardwareMap);

        driveThread = new Thread(){
            @Override
            public void run(){
                while(opModeIsActive()){
                    driveUpdate();
                }
            }
        };

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery

        // Wait for the game to start (driver presses PLAY)

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        wc = hardwareMap.get(WebcamName.class, "Webcam");

        // W/ or W/ out live preview
        camera = OpenCvCameraFactory.getInstance().createWebcam(wc, cameraMonitorViewId);
        // camera = OpenCvCameraFactory.getInstance().createWebcam(wc);

        camera.setPipeline(brp);

        // Open an asynchronous connection to the device
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {

            // Start opening the camera and stream it
            @Override
            public void onOpened() {

                /*
                // create a rgb2gray mat pipeline
                class GrayPipeline extends OpenCvPipeline {
                    Mat gray = new Mat();
                    @Override
                    public Mat processFrame(Mat input) {
                        // mat src, mat dst, int code, convert rgb img to gray
                        Imgproc.cvtColor(input, gray, Imgproc.COLOR_RGB2GRAY);
                        return gray;
                    }
                } */

                camera.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }

            // Method will be called if the camera cannot be opened
            @Override
            public void onError(int errorCode) {
                telemetry.addData("Camera Init Error", errorCode);
            }
        });

        waitForStart();
        runtime.reset();

        driveThread.start();

        //drive.goToPosition(2000, drive.ROTATION);


        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Show the elapsed game time and wheel power.
            capstonePos = brp.getPos();

        }
    }
    public void driveUpdate() {
        //telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower)

        telemetry.addData("Left Tics", "Tics: " + drive.lf.getCurrentPosition());
        telemetry.addData("Right Tics", "Tics: " + drive.rf.getCurrentPosition());
        telemetry.addData("Power", "Power: " + drive.power);
        telemetry.update();


    }
}
