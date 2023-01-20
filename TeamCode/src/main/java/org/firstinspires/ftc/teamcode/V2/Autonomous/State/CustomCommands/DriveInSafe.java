package org.firstinspires.ftc.teamcode.V2.Autonomous.State.CustomCommands;

import android.graphics.Color;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.League1.Subsystems.EndgameSystems;
import org.firstinspires.ftc.teamcode.League1.Subsystems.MecDrive;
import org.firstinspires.ftc.teamcode.V2.NewSubsystem.MecDriveV2;

public class DriveInSafe extends CommandBase {



    MecDriveV2 drive;
    ColorRangeSensor distance;
    boolean isBlue;
    double power;
    ElapsedTime time;


    public DriveInSafe(MecDriveV2 drive, ColorRangeSensor distance, boolean isBlue, double power){
        this.drive = drive;
        this.distance = distance;
        this.isBlue = isBlue;
        this.power = power;

        distance.setGain(300);

    }

    @Override
    public void initialize() {
        time = new ElapsedTime();
        double startTime = time.seconds();



        if(isBlue){
            while (distance.getNormalizedColors().blue < 0.85 && time.seconds() - startTime < 2) {
                drive.setPowerAuto(power, MecDriveV2.MovementType.STRAIGHT);

            }

        }else{
            while (distance.getNormalizedColors().red < 0.7 && time.seconds() - startTime < 2) {
                drive.setPowerAuto(power, MecDriveV2.MovementType.STRAIGHT);

            }
        }

        drive.simpleBrake();



    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
