package org.firstinspires.ftc.teamcode.League1.Subsystems;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.V2.NewSubsystem.MecDriveV2;
import org.firstinspires.ftc.teamcode.V2.NewSubsystem.ScoringSystemV2EpicLift;

public class MotionProfiler {

    MecDriveV2 drive;
    ScoringSystemV2EpicLift score;
    Telemetry telemetry;

    public MotionProfiler(MecDriveV2 drive, ScoringSystemV2EpicLift score, Telemetry telemetry){
        this.drive = drive;
        this.score = score;
        this.telemetry = telemetry;
    }

    public void trapezoidalServoProfile(int milliseconds, double startPos, double targetPos){
        double distance = targetPos - startPos;
        double averageVel = (distance / milliseconds);
        double maxVel = 1.5 * averageVel;

        ElapsedTime time = new ElapsedTime();
        double currentTime = time.milliseconds();

        double position = startPos;

        while(currentTime < milliseconds){
            if(currentTime < (milliseconds / 3)){
                position = (((3 * maxVel) / (2 * milliseconds)) * (Math.pow(currentTime, 2))) + startPos;

            }else if(currentTime < (2 * milliseconds / 3)){
                position = (maxVel * currentTime) + (((3 * maxVel) / (2 * milliseconds)) * (Math.pow((milliseconds/3), 2))) - ((milliseconds/3) * maxVel) + startPos;

            }else{
                position = (((-3 * maxVel) / (2 * milliseconds)) * (Math.pow((currentTime - (milliseconds/3)), 2))) + ((2 * maxVel * currentTime) - distance) + startPos;

            }

            score.setLinkagePosition(position);
            score.addToLoggingString("(" + currentTime + ", " + (position * 1000) + ")");
            currentTime = time.milliseconds();

        }

        score.setLinkagePosition(targetPos);

        score.writeLoggerToFile();
    }


    public void trapezoidalServoProfile(int milliseconds, double targetPos){
        double startPos = score.getRightLinkage();
        double distance = targetPos - startPos;
        double averageVel = (distance / milliseconds);
        double maxVel = 1.5 * averageVel;

        ElapsedTime time = new ElapsedTime();
        double currentTime = time.milliseconds();

        double position = startPos;

        while(currentTime < milliseconds){
            if(currentTime < (milliseconds / 3)){
                position = (((3 * maxVel) / (2 * milliseconds)) * (Math.pow(currentTime, 2))) + startPos;

            }else if(currentTime < (2 * milliseconds / 3)){
                position = (maxVel * currentTime) + (((3 * maxVel) / (2 * milliseconds)) * (Math.pow((milliseconds/3), 2))) - ((milliseconds/3) * maxVel) + startPos;

            }else{
                position = (((-3 * maxVel) / (2 * milliseconds)) * (Math.pow((currentTime - (milliseconds/3)), 2))) + ((2 * maxVel * currentTime) - distance) + startPos;

            }

            score.setLinkagePosition(position);
            score.addToLoggingString("(" + currentTime + ", " + (position * 1000) + ")");
            currentTime = time.milliseconds();

        }

        score.setLinkagePosition(targetPos);

        score.writeLoggerToFile();
    }


    public void driveTrapezoidalProfile(int milliseconds, int distance){
        double averageVel = (Math.abs(distance) / milliseconds);
        double maxVel = 1.5 * averageVel;

        ElapsedTime time = new ElapsedTime();
        double currentTime = time.milliseconds();

        double velocity = 0;

        while(currentTime < milliseconds){
            if(currentTime < (milliseconds / 3)){
                velocity = ((3 * maxVel) / milliseconds) * currentTime;

            }else if(currentTime < (2 * milliseconds / 3)){
                velocity = maxVel;

            }else{
                velocity = (((-3 * maxVel) / milliseconds) * (currentTime - milliseconds));

            }

            velocity += 50;

            if(distance < 0){
                velocity *= -1;
            }

            drive.setVelocity(velocity, velocity, velocity, velocity);

            telemetry.addData("Vel", drive.getAvgVel());
            telemetry.update();
            score.addToLoggingString("(" + currentTime + ", " + (drive.getAvgVel() * 300) + ")");
            currentTime = time.milliseconds();

        }

        drive.simpleBrake();
        score.writeLoggerToFile();

    }



}
