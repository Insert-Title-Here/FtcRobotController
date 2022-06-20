package org.firstinspires.ftc.teamcode.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.ColorSensor;

import org.firstinspires.ftc.teamcode.Intake;
import org.firstinspires.ftc.teamcode.MecanumDriveTrain;
import org.firstinspires.ftc.teamcode.OpModeWrapper;

import java.io.FileNotFoundException;

@Autonomous(name = "Color Stone Auto")
public class ColorStoneAuto extends OpModeWrapper {

    Thread intakeThread;
    MecanumDriveTrain drive;
    Intake intake;
    ColorSensor color;

    @Override
    protected void onInitialize() throws FileNotFoundException{
        intake = new Intake(hardwareMap);
        drive = new MecanumDriveTrain(hardwareMap);
        color = hardwareMap.get(ColorSensor.class, "color");

        intakeThread = new Thread(){
            @Override
            public void run() {


                while(color.red() < 400 && color.blue() < 150 && color.green() < 500){
                    intake.setPower(true, 0.5);
                }
            }
        };


    }

    @Override
    protected void onStart() {

        for(int i = 0; i < 4; i++) {

            while (!(color.red() > 70 && color.blue() > 40 && color.green() > 100)) {
                drive.setPower(0.3, 0.3, 0.3, 0.3);
            }

            intake.clampAndRelease(true);

            intakeThread.start();

            drive.setPower(0.2, 0.2, 0.2, 0.2);
            sleep(1000);

            drive.tankRotate(Math.PI / 2, 0.3);
            drive.driveAuto(0.4, 1500, MecanumDriveTrain.MovementType.STRAIGHT);

        }



    }

    @Override
    protected void onStop() {

    }
}
