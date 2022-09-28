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
        // TODO
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
        //score.setClawPosition(0.4);


        //lift claw a little bit
        score.goToPosition(30, 1);
        // move forward a bit so strafing works better
        drive.goToPosition(-0.3, -0.3, 0.3, 0.3, 130, "forward a bit");
        //strafe right
        drive.goToPosition(-0.3, 0.3, -0.3, 0.3, 640, "strafe right");
        // move forward
        drive.goToPosition(-0.3, -0.3, 0.3, 0.3, 1100, "move forward");
        // strafe left and push cone to other grid
        drive.goToPosition(0.3, -0.3, 0.3, -0.3, 590, "strafe left (push cone)");




        sleep(50);

       /*
        drive.goToPosition(0, 0, 0, 0, 0);
        drive.goToPosition(0, 0, 0, 0, 0);
        drive.goToPosition(0, 0, 0, 0, 0);
        drive.goToPosition(0, 0, 0, 0, 0);
        drive.goToPosition(0, 0, 0, 0, 0);
        drive.goToPosition(0, 0, 0, 0, 0);
        drive.goToPosition(0, 0, 0, 0, 0);

        */






    }

}
