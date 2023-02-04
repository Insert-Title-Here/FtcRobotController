package org.firstinspires.ftc.teamcode.Testing.SubsystemsTest.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.internal.android.dx.dex.code.DalvCode;
import org.firstinspires.ftc.teamcode.Competition.State.Subsystems.Current.MecDriveV2;

@TeleOp
public class EncoderTest extends LinearOpMode {
    DcMotor encoder;
    @Override
    public void runOpMode() throws InterruptedException {

        encoder = hardwareMap.get(DcMotor.class, "fl");


        waitForStart();

        while(opModeIsActive()){
            telemetry.addData("tics", encoder.getCurrentPosition());
            telemetry.update();
        }
    }
}
