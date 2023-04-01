package org.firstinspires.ftc.teamcode.roadrunnerfiles.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.roadrunnerfiles.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunnerfiles.drive.StandardTrackingWheelLocalizer;
import org.firstinspires.ftc.teamcode.roadrunnerfiles.util.Encoder;

import java.util.List;

/**
 * This is a simple teleop routine for testing localization. Drive the robot around like a normal
 * teleop routine and make sure the robot's estimated pose matches the robot's actual pose (slight
 * errors are not out of the ordinary, especially with sudden drive motions). The goal of this
 * exercise is to ascertain whether the localizer has been configured properly (note: the pure
 * encoder localizer heading may be significantly off if the track width has not been tuned).
 */
@Autonomous(group = "drive")
//@Disabled

public class LocalizationTest extends LinearOpMode {

    /*Encoder leftEncoder, rightEncoder, frontEncoder;*/
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.resetEncoders();
        /*StandardTrackingWheelLocalizer thing = new StandardTrackingWheelLocalizer(hardwareMap);
        thing.resetEncoders();*/
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        /*leftEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "LeftLift2"));
        rightEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "RightLift2"));
        frontEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "BackLeftDrive"));
*/
        waitForStart();

        //drive.pid(100, 4, 3);

        while (!isStopRequested()) {
            /*drive.setWeightedDrivePower(
                    new Pose2d(
                            gamepad1.left_stick_y / 2,
                            gamepad1.left_stick_x / 2,
                            gamepad1.right_stick_x / 2
                    )
            );

            drive.update();*/

            List<Double> positions = drive.getWheelPositionsLocalizer();
            //Pose2d poseEstimate = drive.getPoseEstimate();
            /*telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.addData("rightEncoder", rightEncoder.getCurrentPosition());
            telemetry.addData("leftEncoder", leftEncoder.getCurrentPosition());
            telemetry.addData("frontEncoder",frontEncoder.getCurrentPosition());
*/
            telemetry.addData("Other rightEncoder", positions.get(1));
            telemetry.addData("Other leftEncoder", positions.get(0));
            telemetry.addData("Other frontEncoder",positions.get(2));
            telemetry.update();
        }
    }
}
