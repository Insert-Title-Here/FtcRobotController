package org.firstinspires.ftc.teamcode.Testing.Odometry;

////import com.acmerobotics.dashboard.FtcDashboard;
////import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Competition.Interleagues.Common.OpModeWrapper;

@Config

@Autonomous
public class ServoEncoderTest extends LinearOpMode {


    public static double position = 0;
    AnalogInput encoder;
    CRServo servo;

    @Override
    public void runOpMode() throws InterruptedException {

        encoder = hardwareMap.get(AnalogInput.class, "AbsEncoder");
        servo = hardwareMap.get(CRServo.class, "servo");


        waitForStart();
        ElapsedTime time = new ElapsedTime();
        double start = time.seconds();



        while(opModeIsActive()){
            telemetry.addData("Status", encoder.getVoltage());
            telemetry.update();
            //servo.setPower(0.1);

            sleep(1000);
            if(time.seconds() - start > 10){
                //servo.setPower(0);
                break;
            }
        }

    }

    public double getRadians() {
        return encoder.getVoltage() * 1.962;
    }
    public int getDegrees() {
        return (int) (encoder.getVoltage() * 112.39463);
    }


}
