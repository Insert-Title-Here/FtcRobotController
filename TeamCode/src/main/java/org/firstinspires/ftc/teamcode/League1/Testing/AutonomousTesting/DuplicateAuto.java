package org.firstinspires.ftc.teamcode.League1.Testing.AutonomousTesting;

////import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.League1.Autonomous.Vision.KevinGodPipeline;
import org.firstinspires.ftc.teamcode.League1.Common.Constants;
import org.firstinspires.ftc.teamcode.League1.Subsystems.MecDrive;
import org.firstinspires.ftc.teamcode.League1.Subsystems.ScoringSystem2;
import org.firstinspires.ftc.teamcode.V2.NewSubsystem.MecDriveV2;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.concurrent.atomic.AtomicBoolean;

@Disabled
@Autonomous
public class DuplicateAuto extends LinearOpMode {
    MecDriveV2 drive;




    @Override
    public void runOpMode() throws InterruptedException {
        drive = new MecDriveV2(hardwareMap, false, telemetry, true);



        waitForStart();

        drive.simpleMoveToPosition(-100, MecDriveV2.MovementType.ROTATE, 0.5);

        sleep(500);

        drive.simpleMoveToPosition(-17, MecDriveV2.MovementType.ROTATE, 0.5);

        sleep(500);

        drive.simpleMoveToPosition(-5, MecDriveV2.MovementType.ROTATE, 0.5);

        sleep(2000);

        CommandScheduler.getInstance().schedule(
                new InstantCommand(() -> drive.simpleMoveToPosition(-100, MecDriveV2.MovementType.ROTATE, 0.5)),
                new InstantCommand(() -> sleep(500)),
                new InstantCommand(() -> drive.simpleMoveToPosition(-17, MecDriveV2.MovementType.ROTATE, 0.5)),
                new InstantCommand(() -> sleep(500)),
                new InstantCommand(() -> drive.simpleMoveToPosition(-5, MecDriveV2.MovementType.ROTATE, 0.5)),
                new InstantCommand(() -> sleep(500))
        );



        CommandScheduler.getInstance().run();




    }



}