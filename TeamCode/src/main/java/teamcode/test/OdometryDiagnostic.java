package teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayList;

import teamcode.common.AbstractOpMode;
import teamcode.common.Localizer;
import teamcode.common.MecanumDriveTrain;
import teamcode.common.PurePursuit.CurvePoint;
import teamcode.common.PurePursuit.MovementVars;
import teamcode.common.PurePursuit.PurePursuitMovement;
import teamcode.common.Vector2D;
import teamcode.common.WestCoastDriveTrain;

@Disabled
@TeleOp(name="OdoPushTest")
public class OdometryDiagnostic extends AbstractOpMode {

    WestCoastDriveTrain drive;
    Localizer localizer;

    @Override
    protected void onInitialize() {
        localizer = new Localizer(hardwareMap, new Vector2D(0,0), 0, 10);
        //localizer = new Localizer(hardwareMap, new Vector2D(0,0), 0);
        //movement = new PurePursuitMovement(localizer);
        //allPoints.add(new CurvePoint(25, 0, 0.5, 0.3, 5, 0, 1));
        //allPoints.add(new CurvePoint(25, 25, 0.5, 0, 5, 0, 1));
        //allPoints.add(new CurvePoint(0, 25, 0.5, 0.3, 5, 0, 1));

    }

    @Override
    protected void onStart() {
        localizer.start();
        while(opModeIsActive()){
//            telemetry.addData("Position", localizer.getCurrentState().getPosition());
//            telemetry.addData("Rotation", localizer.getCurrentState().getRotation());

            //movement.followCurve(allPoints, 0);

            //telemetry.addData("CurrentIndex", movement.getCurrentRobotIndex());
//            telemetry.update();
        }
    }

    @Override
    protected void onStop() {
        localizer.loggingString += localizer.getCurrentState().toString();
        localizer.writeLoggerToFile();
        localizer.stopThread();


    }
}
