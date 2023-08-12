package org.firstinspires.ftc.teamcode.Competition.Interleagues.Autonomous.AttemptsToFixAuto;

////import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Competition.State.Subsystems.Current.MecDriveV2;

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