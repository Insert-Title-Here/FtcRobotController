package org.firstinspires.ftc.teamcode.Competition.State.Autonomous.InUse.CustomCommands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Competition.State.Subsystems.Current.MecDriveV2;

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
