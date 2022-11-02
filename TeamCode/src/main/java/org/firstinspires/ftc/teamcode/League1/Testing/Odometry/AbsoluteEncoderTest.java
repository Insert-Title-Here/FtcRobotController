package org.firstinspires.ftc.teamcode.League1.Testing.Odometry;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.League1.Autonomous.Vision.KevinGodPipeline;
import org.firstinspires.ftc.teamcode.League1.Autonomous.Vision.SignalPipeline;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;


@Autonomous
public class AbsoluteEncoderTest extends LinearOpMode {

    AnalogInput encoder;

    @Override
    public void runOpMode() throws InterruptedException {

        double max = 0;

        encoder = hardwareMap.get(AnalogInput.class, "AbsEncoder");

        waitForStart();

        while(opModeIsActive()){
            if(encoder.getVoltage() > max) {
                max = encoder.getVoltage();
            }
            telemetry.addData("Max", max);
            telemetry.addData("Encoder Voltage", encoder.getVoltage());
            telemetry.update();
        }
        // 3.203 = max = 2pi
    }
}
