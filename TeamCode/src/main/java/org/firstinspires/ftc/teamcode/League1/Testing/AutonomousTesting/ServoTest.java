package org.firstinspires.ftc.teamcode.League1.Testing.AutonomousTesting;

//import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator;
import com.acmerobotics.roadrunner.profile.MotionState;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;


import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.League1.Autonomous.Vision.KevinGodPipeline;
import org.firstinspires.ftc.teamcode.League1.Autonomous.Vision.SignalPipeline;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

@Disabled
@Autonomous
public class ServoTest extends LinearOpMode {


    MotionProfile profile;
    ServoImplEx lLinkage;
    ServoImplEx rLinkage;

    @Override
    public void runOpMode() throws InterruptedException {



        //profile = MotionProfileGenerator.generateMotionProfile(new MotionState(0.05, 0), new MotionState(0.7, 0), new Ve, 0.01);
        lLinkage = hardwareMap.get(ServoImplEx.class, "LeftLinkage");
        rLinkage = hardwareMap.get(ServoImplEx.class, "RightLinkage");

        lLinkage.setPwmRange(new PwmControl.PwmRange(500, 2500));
        rLinkage.setPwmRange(new PwmControl.PwmRange(500, 2500));

        lLinkage.setPosition(0);
        rLinkage.setPosition(0);

        waitForStart();

        for(int i = 0; i < 5; i++) {
            lLinkage.setPosition(0);
            rLinkage.setPosition(0);

            sleep(2000);

            lLinkage.setPosition(1);
            rLinkage.setPosition(1);

            sleep(2000);
        }


    }
}
