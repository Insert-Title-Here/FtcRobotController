package org.firstinspires.ftc.teamcode.Testing.MotionProfile;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Competition.Interleagues.Common.Constants;
import org.firstinspires.ftc.teamcode.Competition.State.Subsystems.Current.MotionProfiler;
import org.firstinspires.ftc.teamcode.Competition.State.Subsystems.Current.MecDriveV2;
import org.firstinspires.ftc.teamcode.Competition.State.Subsystems.Current.ScoringSystemV2EpicLift;

@Disabled
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
