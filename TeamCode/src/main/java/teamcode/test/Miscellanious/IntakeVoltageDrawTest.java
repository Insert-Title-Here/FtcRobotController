package teamcode.test.Miscellanious;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

import teamcode.Competition.Subsystems.ArmSystem;
import teamcode.common.AbstractOpMode;
import teamcode.common.Logger;
import teamcode.common.MecanumDriveTrain;

@Autonomous(name="voltage")
public class IntakeVoltageDrawTest extends AbstractOpMode {
    MecanumDriveTrain drive;
    ArmSystem system;
    @Override
    protected void onInitialize() {
        system = new ArmSystem(hardwareMap, false);
        drive = new MecanumDriveTrain(hardwareMap, true, null, system, new PIDFCoefficients(2.5, 0.5, 1.0 ,0));
    }

    int iterator;
    @Override
    protected void onStart() {
        drive.driveColorSensor(1);
        while(opModeIsActive());
    }

    @Override
    protected void onStop() {

    }
}
