//Auto for if we are just scoring preset

package org.firstinspires.ftc.teamcode.Autonomous.TestAutos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Common.Constants;
import org.firstinspires.ftc.teamcode.Common.MecDriveV2;
import org.firstinspires.ftc.teamcode.Common.ScoringSystem;

@Autonomous
public class RedBackAutoTest extends LinearOpMode {

    MecDriveV2 drive;
    ScoringSystem score;
    ElapsedTime time;

    public void runOpMode() {

        time = new ElapsedTime();
        drive = new MecDriveV2(hardwareMap, false, telemetry, time);
        score = new ScoringSystem(hardwareMap, telemetry, time);
        score.setGrabberPosition(Constants.OPEN);
        score.setGrabberPosition(Constants.GRABBING);
        int rando = 0;
        //vision stuff to assign 1, 2, or 3 to rando

        waitForStart();
        if (rando == 1) {
            random1();
        }
        else if (rando == 2) {
            random2();
        }

        drive.simpleBrake();
        sleep(500);

    }
    public void random1() {
        drive.simpleMoveToPosition(600, 0.4);
        drive.tankRotate(Math.PI/2, 0.3);

        score.setIntakePower(-0.2);
        sleep(500);
        score.setIntakePower(0);

        drive.simpleMoveToPosition(-650, 0.4);
        drive.tankRotate(3*Math.PI/2, 0.3);
        drive.simpleMoveToPosition(-125, 0.1);

        autoScore();

        drive.simpleMoveToPosition(100, 0.3);
        drive.simpleMoveToPosition(600, MecDriveV2.MovementType.STRAFE, 0.4);

    }
    public void random2 () {
        drive.simpleMoveToPosition(635, 0.4);
        score.setIntakePower(-0.2);
        sleep(500);
        score.setIntakePower(0);
        drive.simpleMoveToPosition(-50, 0.2);

        drive.tankRotate((Math.PI/2), 0.3);

        drive.simpleMoveToPosition(-650, 0.4);
        drive.simpleMoveToPosition(-145, 0.3);
        drive.simpleMoveToPosition(-125, 0.1);

        //  score.
        autoScore();

        drive.simpleMoveToPosition(100, MecDriveV2.MovementType.STRAIGHT, 0.3);

        drive.simpleMoveToPosition(650, MecDriveV2.MovementType.STRAFE, 0.4);
    }

    public void random3() {


    }

    public void autoScore() {
        score.setLinkagePositionLogistic(0.5, 1000, 100);
        //  score.goToLiftTarget(Constants.LIFT_LOW, 0.8);
        score.setLinkagePositionLogistic(Constants.LINKAGE_UP, 1500, 100);
        sleep(300);
        score.setGrabberPosition(Constants.OPEN);
        sleep(300);
        score.setLinkagePositionLogistic(0.5, 1000, 100);
        //   score.goToLiftTarget(0, 0.3);
        score.setLinkagePositionLogistic(Constants.LINKAGE_DOWN, 1000, 100);
    }


}
