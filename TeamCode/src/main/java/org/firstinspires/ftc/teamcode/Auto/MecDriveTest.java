package org.firstinspires.ftc.teamcode.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Common.MecanumDrive;
import org.firstinspires.ftc.teamcode.Common.ScoringSystem;

@Autonomous
public class MecDriveTest extends LinearOpMode {
    MecanumDrive drive;
    ScoringSystem score;
    int parkLocation;
    @Override
    public void runOpMode() throws InterruptedException {
        drive = new MecanumDrive(hardwareMap, telemetry);
        score = new ScoringSystem(hardwareMap);
        // TODO: Fix "User OpMode was stuck in stop(), but was able to be force stopped s/o restarting the app. It appears
        // TODO: Continued... this awas a linear OPMODE; wmake sure you are calling opModelIsActive() in any loops.
        /*
        -put cone on pole, get new cone, repeat
        -read custom sleeve, remember place to park accordingly
         */

        //Example Code for Direction
        /*
        //forward
        drive.goToPosition(0.3, 0.3, 0.3, 0.3, 5000);

        //backward
        drive.goToPosition(-0.3, -0.3, -0.3, -0.3, 5000);

        //Left
        drive.goToPosition(0.3, -0.3, 0.3, -0.3, 5000);

        //Right
        drive.goToPosition(-0.3, 0.3, 0.3, -0.3, 5000);

        //Forward-Left Diagonal
        drive.goToPosition(0.3, 0, 0, 0.3, 3000);

        //Forward-Right Diagonal
        drive.goToPosition(0, 0.3, 0.3, 0, 3000);

        //Backward-Left Diagonal
        drive.goToPosition(-0.3, 0, 0, -0.3, 3000);

        //Backward-Right Diagonal
        drive.goToPosition(0, -0.3, -0.3, 0, 3000);
         */

        // Camera checks sleeve...stores parking location??

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();
        //close claw
        score.setClawPosition(0.48);


        //lift claw a little bit
        score.goToPosition(30, 0.3);
        // move forward a square
        drive.goToPosition(-0.3, -0.3, 0.3, 0.3, avgPosition(-1235, -1198, 1204, 1144)/4, "forward");
        sleep(50);
        //strafe left
        drive.goToPosition(-0.3, 0.3, -0.3, 0.3, avgPosition(1448, -1398, 1598, -1500), "strafe right");
        sleep(50);
        // turn -> 406, -397, -438, 440
        // move arm max
        score.goToPosition(2390, 1);

        drive.goToPosition(-0.3, -0.3, 0.3, 0.3, avgPosition(-326, -300, 330, 304), "move to pole");
        score.setClawPosition(0.9);
        score.goToPosition(0, 0.5);
        sleep(50);

        //1 (far left) (general code)
        drive.goToPosition(0.3, 0.3, -0.3, -0.3, avgPosition(498, 506, -557, -565), "move back from pole");
        sleep(50);
        //turn -> -395, 368, 413, -410
        drive.goToPosition(-0.3, -0.3, 0.3, 0.3, avgPosition(-98, -95, 105, 92), "move forward a bit");
        sleep(50);

        /*
        //2
        drive.goToPosition(-0.3, 0.3, -0.3, 0.3, avgPosition(-1267, 1251, -1246, 304), "strafe right");
        sleep(50);
        //3
        drive.goToPosition(-0.3, 0.3, -0.3, 0.3, avgPosition(-1152, 1177, -1164, 1196), "strafe right");

        sleep(50);
    */






    }
    public int avgPosition(int fl, int fr, int bl, int br){
        return (int)(Math.abs(fl) + Math.abs(fr) + Math.abs(bl) + Math.abs(br))/4;
    }

}
