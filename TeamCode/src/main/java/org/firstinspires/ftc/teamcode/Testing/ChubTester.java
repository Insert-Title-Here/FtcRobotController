package org.firstinspires.ftc.teamcode.Testing;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Competition.Interleagues.Common.Constants;

@TeleOp (name = "Testing CHUBS")
public class ChubTester extends LinearOpMode {

    DcMotor motor0, motor1, motor2, motor3;
    Servo servo0, servo1, servo2, servo3, servo4, servo5;

    @Override
    public void runOpMode() throws InterruptedException {

        servo0 = hardwareMap.get(Servo.class, "servo");
        servo1 = hardwareMap.get(Servo.class, "servo");
        servo2 = hardwareMap.get(Servo.class, "servo");
        servo3 = hardwareMap.get(Servo.class, "servo");
        servo4 = hardwareMap.get(Servo.class, "servo");
        servo5 = hardwareMap.get(Servo.class, "servo");
        motor0 = hardwareMap.get(DcMotor.class, "motor0");
        motor1 = hardwareMap.get(DcMotor.class, "motor1");
        motor2 = hardwareMap.get(DcMotor.class, "motor2");
        motor3 = hardwareMap.get(DcMotor.class, "motor3");



        waitForStart();


        while(opModeIsActive()){
            //Forwards
            motor0.setPower(0.1);
            motor1.setPower(0.1);
            motor2.setPower(0.1);
            motor3.setPower(0.1);

            //Servo Position = 0
            servo0.setPosition(0);
            servo1.setPosition(0);
            servo2.setPosition(0);
            servo3.setPosition(0);
            servo4.setPosition(0);
            servo5.setPosition(0);

            sleep(2000);

            //Telemetry (encoder position)
            telemetry.addData("encoder position 0", motor0.getCurrentPosition());
            telemetry.addData("encoder position 1", motor1.getCurrentPosition());
            telemetry.addData("encoder position 2", motor2.getCurrentPosition());
            telemetry.addData("encoder position 3", motor3.getCurrentPosition());

            //Backwards
            motor0.setPower(-0.1);
            motor1.setPower(-0.1);
            motor2.setPower(-0.1);
            motor3.setPower(-0.1);

            //Servo Position = 1
            servo0.setPosition(1);
            servo1.setPosition(1);
            servo2.setPosition(1);
            servo3.setPosition(1);
            servo4.setPosition(1);
            servo5.setPosition(1);
            sleep(2000);

            telemetry.update();


        }

        motor0.setPower(0);
        motor1.setPower(0);
        motor2.setPower(0);
        motor3.setPower(0);
        servo0.setPosition(0);
        servo1.setPosition(0);
        servo2.setPosition(0);
        servo3.setPosition(0);
        servo4.setPosition(0);
        servo5.setPosition(0);
        sleep(1000);

    }
}

