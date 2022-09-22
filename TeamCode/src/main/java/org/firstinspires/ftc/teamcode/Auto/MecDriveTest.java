package org.firstinspires.ftc.teamcode.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Common.MecanumDrive;

@Autonomous
public class MecDriveTest extends LinearOpMode {
    MecanumDrive drive;
    int parkLocation;
    @Override
    public void runOpMode() throws InterruptedException {
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
        waitForStart();

        drive.goToPosition(0, 0.3, 0.3, 0, 4000);
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
