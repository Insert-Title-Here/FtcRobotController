//Auto for if we are just scoring preload
//mostly done?

package org.firstinspires.ftc.teamcode.Autonomous.TestMeAutos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Const;
import org.firstinspires.ftc.teamcode.Common.Constants;
import org.firstinspires.ftc.teamcode.Common.MecDriveV2;
import org.firstinspires.ftc.teamcode.Common.ScoringSystem;
//@Disabled
@Autonomous
public class RedBackAutoTest extends LinearOpMode {

    MecDriveV2 drive;
    ScoringSystem score;
    ElapsedTime time;

    public void runOpMode() {

        time = new ElapsedTime();
        drive = new MecDriveV2(hardwareMap, false, telemetry, time);
        score = new ScoringSystem(hardwareMap, telemetry, time);
        score.setGrabberPosition(Constants.GRABBING);
        score.setGrabberPosition(Constants.AUTO_SCORING_CLAMP_OPEN);
        score.setIntakeLiftPos(Constants.INTAKE_LINKAGE_UP);

        int rando = 2;
        //vision stuff to assign 1, 2, or 3 to rando7 0

        waitForStart();
        score.setIntakeLiftPos(Constants.INTAKE_LINKAGE_UP);
        score.setBumperPixelRelease(Constants.AUTO_SCORING_CLAMP_CLOSED);
        sleep(1500);

        if (rando == 1) {
            random1();
        }
        else if (rando == 2) {
            random2();
        }
        else if (rando == 3) {
            random3();
        }

        drive.simpleBrake();
        sleep(500);

    }
    public void random1() {
        drive.simpleMoveToPosition(-600, Constants.AUTO_LINEAR_SPEED);
        drive.tankRotate(Math.PI/(2), Constants.AUTO_ROTATIONAL_SPEED);

        spikeScore();
        /*
        score.setIntakePower(-0.2);
        sleep(500);
        score.setIntakePower(0);
      */
        drive.simpleMoveToPosition(200, Constants.AUTO_SAFE_MO);
        drive.tankRotate(Math.PI/(-2.1), Constants.AUTO_ROTATIONAL_SPEED);

        drive.simpleMoveToPosition(-750, Constants.AUTO_LINEAR_SPEED);

        creep();
        autoScore();

        drive.simpleMoveToPosition(200, MecDriveV2.MovementType.STRAIGHT, Constants.AUTO_LINEAR_SPEED);
        drive.simpleMoveToPosition(600, MecDriveV2.MovementType.STRAFE, Constants.AUTO_LINEAR_SPEED);
        drive.simpleMoveToPosition(-300, Constants.AUTO_LINEAR_SPEED);
    }
    public void random2 () {
        drive.simpleMoveToPosition(-575, Constants.AUTO_LINEAR_SPEED);

        spikeScore();
        /*
        score.setIntakePower(-0.2);
        sleep(500);
        score.setIntakePower(0);

         */
        drive.simpleMoveToPosition(150, Constants.AUTO_SAFE_MO);

        drive.tankRotate((Math.PI/(-2.15)), Constants.AUTO_ROTATIONAL_SPEED);

        drive.simpleMoveToPosition(-650, Constants.AUTO_LINEAR_SPEED);
        drive.simpleMoveToPosition(-250, Constants.AUTO_SAFE_MO);

        creep();
        autoScore();

        drive.simpleMoveToPosition(200, MecDriveV2.MovementType.STRAIGHT, Constants.AUTO_LINEAR_SPEED);
        drive.simpleMoveToPosition(625, MecDriveV2.MovementType.STRAFE, Constants.AUTO_LINEAR_SPEED);
        drive.simpleMoveToPosition(-200, Constants.AUTO_LINEAR_SPEED);
        score.setLinkagePositionLogistic(Constants.INTAKE_LINKAGE_UP-0.2, 2000);


    }

    public void random3() {
        drive.simpleMoveToPosition(-50, Constants.AUTO_LINEAR_SPEED);
        drive.simpleMoveToPosition(-325, MecDriveV2.MovementType.STRAFE, Constants.AUTO_LINEAR_SPEED);
        drive.simpleMoveToPosition(-200, Constants.AUTO_LINEAR_SPEED);

        spikeScore();
        /*
        score.setIntakePower(-0.2);
        sleep(500);
        score.setIntakePower(0);

         */
        drive.simpleMoveToPosition(100, Constants.AUTO_LINEAR_SPEED);
        drive.simpleMoveToPosition(-450, MecDriveV2.MovementType.STRAFE, Constants.AUTO_LINEAR_SPEED);
        drive.simpleMoveToPosition(-250, MecDriveV2.MovementType.STRAIGHT, Constants.AUTO_LINEAR_SPEED);
        drive.tankRotate(Math.PI/(-2), Constants.AUTO_ROTATIONAL_SPEED);
        drive.simpleMoveToPosition(-250, MecDriveV2.MovementType.STRAIGHT, Constants.AUTO_LINEAR_SPEED);

        creep();
        autoScore();
        drive.simpleMoveToPosition(200, MecDriveV2.MovementType.STRAIGHT, Constants.AUTO_LINEAR_SPEED);
        drive.simpleMoveToPosition(600, MecDriveV2.MovementType.STRAFE, Constants.AUTO_LINEAR_SPEED);
        drive.simpleMoveToPosition(-400, Constants.AUTO_LINEAR_SPEED);
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

    public void creep() {
        drive.setPowerAuto(Constants.AUTO_SLOWED_SPEED, MecDriveV2.MovementType.STRAIGHT);
        //Change me!
        sleep(600);
        drive.setPowerAuto(0, MecDriveV2.MovementType.STRAIGHT);
    }

    public void spikeScore() {
        drive.simpleMoveToPosition(-100, Constants.AUTO_SAFE_MO);
        score.setBumperPixelRelease(Constants.AUTO_SCORING_CLAMP_OPEN);
        drive.simpleMoveToPosition(50, Constants.AUTO_SLOWED_SPEED);
        score.setBumperPixelRelease(Constants.AUTO_SCORING_CLAMP_CLOSED);
        drive.simpleMoveToPosition(50, Constants.AUTO_SAFE_MO);

    }


}
