package org.firstinspires.ftc.teamcode.Testing.Odometry;

////import com.acmerobotics.dashboard.FtcDashboard;
////import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Competition.Interleagues.Common.OpModeWrapper;

////@Config
@Disabled
@Autonomous
public class AbsoluteEncoderTest extends LinearOpMode {

    public static double P = 0.001;
    public static double I = 0.00001;
    public static double D = 0.0001;
    public static double F = 0;

    AnalogInput encoder;
    CRServo servo;

    @Override
    public void runOpMode() throws InterruptedException {

        encoder = hardwareMap.get(AnalogInput.class, "AbsEncoder");
        servo = hardwareMap.get(CRServo.class, "servo");

        setPositionSimple(0.1, 50);

        waitForStart();

        setPositionPID(0.1, 120);
        sleep(1000);
        updateTelemetry();
        sleep(1000);
        setPositionPID(0.1, 270);

        while(opModeIsActive()){
            telemetry.addData("Status", "PID Complete");
            updateTelemetry();
        }
        // 3.203 = max = 2pi rads = 360 deg
        // 3.203/1 = xpi/1
    }

    public double getRadians() {
        return encoder.getVoltage() * 1.962;
    }
    public int getDegrees() {
        return (int) (encoder.getVoltage() * 112.39463);
    }

    public void setPositionSimple(double power, int targetDegrees) {
        if(getDegrees() > targetDegrees) {
            while(getDegrees() > targetDegrees) {
                updateTelemetry();
                servo.setPower(-power);
            }
        } else {
            while(getDegrees() < targetDegrees) {
                updateTelemetry();
                servo.setPower(power);
            }
        }
        servo.setPower(0);
    }

    public void setPositionPID(double power, int target) {
        /*
        double P = 0.005;
        double I = 0;
        double D = 0;
        double F = 0;

         */

        ElapsedTime time = new ElapsedTime();
        double startTime = time.seconds();
        boolean countTime = true;
        double pos = getDegrees();

        int error = target - getDegrees();
        int previousError = error;
        double integralSum = 0;
        while(Math.abs(error) > 2) {
            error = target - getDegrees();
            double currentTime = time.seconds();
            telemetry.addData("error", error);
            integralSum += (0.5 * (error + previousError) * (currentTime - startTime));
            telemetry.addData("flIntegralSum", integralSum);

            if(integralSum > 20000){
                integralSum = 20000;
            }else if(integralSum < -20000){
                integralSum = -20000;
            }

            double derivative = (error - previousError)/(currentTime - startTime);

            telemetry.addData("derivative", derivative);

            double servoPower = (P * error) + (I * integralSum) + (D * derivative) + (F * power);
            servo.setPower(servoPower);
            telemetry.addData("servoPower", servoPower);
            telemetry.update();
            startTime = currentTime;
        }
        servo.setPower(0);
    }
    public boolean opModeIsRunning() {
        return OpModeWrapper.currentOpMode().opModeIsActive() && !OpModeWrapper.currentOpMode().isStopRequested();
    }
    public void updateTelemetry() {
        telemetry.addData("Voltage", encoder.getVoltage());
        telemetry.addData("Encoder Rads", getRadians());
        telemetry.addData("Encoder Degrees", getDegrees());
        telemetry.addData("Max", encoder.getMaxVoltage());
        telemetry.update();
    }
}
