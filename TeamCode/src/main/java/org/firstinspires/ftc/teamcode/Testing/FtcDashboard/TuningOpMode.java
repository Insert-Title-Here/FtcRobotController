package org.firstinspires.ftc.teamcode.Testing.FtcDashboard;

/*
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="Tuning OpMode")
public class TuningOpMode extends LinearOpMode {

    static double TEST1_POS = 0.43;
    static double TEST2_POS = 0;

    Servo testServo1 = hardwareMap.get(Servo.class, "RightClamp");
    Servo testServo2 = hardwareMap.get(Servo.class, "Linkage");

    @Override
    public void runOpMode() {

        FtcDashboard dashboard = FtcDashboard.getInstance();
        dashboard.updateConfig();

        TelemetryPacket telemetryStuff = new TelemetryPacket();

        telemetryStuff.put("Servo1", TEST1_POS);
        telemetryStuff.put("Servo2", TEST2_POS);

        dashboard.sendTelemetryPacket(telemetryStuff);

        testServo1.setPosition(TEST1_POS);
        testServo2.setPosition(TEST2_POS);

        while (opModeIsActive()) {

            testServo1.setPosition(TEST1_POS);
            testServo2.setPosition(TEST2_POS);

            telemetryStuff.put("Servo1", TEST1_POS);
            telemetryStuff.put("Servo2", TEST2_POS);
            dashboard.sendTelemetryPacket(telemetryStuff);
            dashboard.updateConfig();

            sleep(50);

        }
    }
}


 */