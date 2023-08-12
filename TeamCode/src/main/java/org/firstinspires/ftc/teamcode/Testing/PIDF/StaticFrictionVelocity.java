package org.firstinspires.ftc.teamcode.Testing.PIDF;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Competition.Interleagues.Common.OpModeWrapper;
import org.firstinspires.ftc.teamcode.Competition.State.Subsystems.Current.MecDriveV2;
import org.firstinspires.ftc.teamcode.Competition.State.Subsystems.Current.ScoringSystemV2EpicLift;

import java.io.FileNotFoundException;

@Disabled

@TeleOp
public class StaticFrictionVelocity extends OpModeWrapper {
    //Servo lServo, rServo, grabber;
    //DcMotor rMotor, lMotor;
    MecDriveV2 drive;
    ScoringSystemV2EpicLift score;
    int flVel, frVel, blVel, brVel, finalFlVel, finalFRVel, finalBlVel, finalBRVel;
    boolean fl, fr, bl, br;


    @Override
    protected void onInitialize() throws FileNotFoundException {

        score = new ScoringSystemV2EpicLift(hardwareMap, telemetry, true);

        drive = new MecDriveV2(hardwareMap, false, telemetry);

        flVel = 0;
        frVel = 0;
        blVel = 0;
        brVel = 0;

        fl = true;
        fr = true;
        bl = true;
        br = true;

    }

    @Override
    protected void onStart() {


        while(opModeIsActive()) {
            drive.setVelocity(flVel, frVel, blVel, brVel);

            telemetry.addData("fl", flVel);
            telemetry.addData("fr", frVel);
            telemetry.addData("bl", blVel);
            telemetry.addData("br", brVel);
            telemetry.update();


            sleep(3000);


            if(drive.getFLVelocity() > 0.2){
                finalFlVel = flVel;
                flVel = 0;
                fl = false;
            }

            if(drive.getFRVelocity() > 0.2){
                finalFRVel = flVel;
                frVel = 0;
                fr = false;
            }

            if(drive.getBLVelocity() > 0.2){
                finalBlVel = flVel;
                blVel = 0;
                bl = false;
            }

            if(drive.getBRVelocity() > 0.2){
                finalBRVel = flVel;
                brVel = 0;
                br = false;
            }


            if(fl){
                flVel += 1;
            }

            if(fr){
                frVel += 1;
            }

            if(bl){
                blVel += 1;
            }

            if(br){
                brVel += 1;
            }



        }





    }

    @Override
    protected void onStop() {
        score.setPower(0);
    }


}
