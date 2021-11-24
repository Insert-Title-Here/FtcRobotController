package org.firstinspires.ftc.teamcode.MecanumCode.Auto;

import org.firstinspires.ftc.teamcode.MecanumCode.Common.MecanumDriveTrain;
import org.firstinspires.ftc.teamcode.MecanumCode.Common.OpModeWrapper;

public class TestAuto extends OpModeWrapper {

    MecanumDriveTrain drive;

    @Override
    protected void onInitialize() {
        drive = new MecanumDriveTrain(hardwareMap);
    }

    @Override
    protected void onStart() {
        drive.driveAuto(120, 240, MecanumDriveTrain.MovementType.STRAIGHT);
        drive.driveAuto(120, 240, MecanumDriveTrain.MovementType.STRAFE);
        drive.driveAuto(120, 240, MecanumDriveTrain.MovementType.ROTATE);
        while(opModeIsActive());
    }

    @Override
    protected void onStop() {

    }
}
