package org.firstinspires.ftc.teamcode.League1.Testing.MotionProfile;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.League1.Common.Constants;
import org.firstinspires.ftc.teamcode.League1.Subsystems.MecDrive;
import org.firstinspires.ftc.teamcode.League1.Subsystems.MotionProfiler;
import org.firstinspires.ftc.teamcode.League1.Subsystems.ScoringSystem2;
import org.firstinspires.ftc.teamcode.V2.NewSubsystem.MecDriveV2;
import org.firstinspires.ftc.teamcode.V2.NewSubsystem.ScoringSystemV2;
import org.firstinspires.ftc.teamcode.V2.NewSubsystem.ScoringSystemV2EpicLift;

//@Disabled
@Autonomous
public class DriveMotionProfile extends LinearOpMode {
    MecDriveV2 drive;
    ScoringSystemV2EpicLift score;
    MotionProfiler profile;


    //For Rotate method (tankRotatePID)


    @Override
    public void runOpMode() throws InterruptedException {
        //constants = newConstants();
        drive = new MecDriveV2(hardwareMap, true, telemetry,true);
        score = new ScoringSystemV2EpicLift(hardwareMap, telemetry, false);

        profile = new MotionProfiler(drive, score, telemetry);

        waitForStart();

        //profile.driveTrapezoidalProfile(5000, 300);
        profile.trapezoidalServoProfile(1000, Constants.linkageDownV2, Constants.linkageUpV2);

        sleep(1000);

        profile.driveTrapezoidalProfile(200, 1000);

        sleep(1000);






    }







}
