package org.firstinspires.ftc.teamcode.Testing.SubsystemsTest.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.internal.android.dx.dex.code.DalvCode;
import org.firstinspires.ftc.teamcode.Competition.MTI.ScoringSystemNewest;
import org.firstinspires.ftc.teamcode.Competition.State.Subsystems.Current.MecDriveV2;

@TeleOp
public class EncoderTest extends LinearOpMode {
    DcMotor fl, fr, bl, br, rl1, rl2, ll1, ll2;
    ScoringSystemNewest score;
    @Override
    public void runOpMode() throws InterruptedException {

        score = new ScoringSystemNewest(hardwareMap, telemetry, true, new ElapsedTime());


        fl = hardwareMap.get(DcMotor.class, "FrontLeftDrive");
        fr = hardwareMap.get(DcMotor.class, "FrontRightDrive");
        bl = hardwareMap.get(DcMotor.class, "BackLeftDrive");
        br = hardwareMap.get(DcMotor.class, "BackRightDrive");

        rl1 = hardwareMap.get(DcMotor.class, "RightLift");
        rl2 = hardwareMap.get(DcMotor.class, "RightLift2");
        ll1 = hardwareMap.get(DcMotor.class, "LeftLift");
        ll2 = hardwareMap.get(DcMotor.class, "LeftLift2");

        score.setGrabberPosition(0.13);


        waitForStart();

        while(opModeIsActive()){
            telemetry.addData("Front Left Tics: ", fl.getCurrentPosition());
            telemetry.addData("Front Right Tics: ", fr.getCurrentPosition());
            telemetry.addData("Back Left Tics: ", bl.getCurrentPosition());
            telemetry.addData("Back Right Tics: ", br.getCurrentPosition());
            telemetry.addData("Right Lift 1 Tics: ", rl1.getCurrentPosition());
            telemetry.addData("Right Lift 2 Tics: ", rl2.getCurrentPosition());
            telemetry.addData("Left Lift 1 Tics: ", ll1.getCurrentPosition());
            telemetry.addData("Left Lift 2 Tics: ", ll2.getCurrentPosition());
            telemetry.update();
        }
    }
}
