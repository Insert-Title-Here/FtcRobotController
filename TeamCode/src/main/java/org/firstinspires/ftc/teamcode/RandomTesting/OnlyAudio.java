package org.firstinspires.ftc.teamcode.RandomTesting;

import android.media.MediaPlayer;
import android.provider.MediaStore;

import com.qualcomm.ftccommon.SoundPlayer;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.R;
import org.openftc.revextensions2.ExpansionHubEx;

import java.io.File;

@TeleOp
public class OnlyAudio extends LinearOpMode {
    //ExpansionHubEx hub;



    Thread ledThread = new Thread() {
        @Override
        public void run() {
            while (opModeIsActive()) {

                int r = (int)(Math.random() * 255);
                int g = (int)(Math.random() * 255);
                int b = (int)(Math.random() * 255);

                //hub.setLedColor(r, g, b);



                try {
                    Thread.currentThread().sleep(100);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }



            }
        }
    };

    @Override
    public void runOpMode() throws InterruptedException {
        //SoundPlayer.getInstance().startPlaying(hardwareMap.appContext, R.raw.amongus);
        //hub = hardwareMap.get(ExpansionHubEx.class, "Control Hub");


        waitForStart();

        //ledThread.start();


        while (opModeIsActive()) {
            if (gamepad1.a) {
                SoundPlayer.getInstance().startPlaying(hardwareMap.appContext, R.raw.amongus);

                sleep(5000);

                SoundPlayer.getInstance().stopPlayingAll();


            }else if(gamepad1.b){
                SoundPlayer.getInstance().startPlaying(hardwareMap.appContext, R.raw.johncena);

                sleep(15000);

                SoundPlayer.getInstance().stopPlayingAll();
            }


        }

        SoundPlayer.getInstance().stopPlayingAll();




    }
}
