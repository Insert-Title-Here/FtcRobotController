package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Testing.RobotK;

public class Lift {

    public DcMotor lift;

    private RobotK robot;
    private Telemetry telemetry;
    public boolean extended;

    public Lift(HardwareMap hardwareMap, RobotK robot, Telemetry telemetry) {
        lift = hardwareMap.get(DcMotor.class, "Lift");

        extended = false;

        this.robot = robot;
        this.telemetry = telemetry;

        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //TODO: See if we need to set direction
    }

    public void setPower(double power) {
        lift.setPower(power);
    }

    public void runToPosition(int tics, double power) {
        while (Math.abs(tics - robot.getSpecificEncoderValue(0, false)) > 10) {
            robot.update();
            //telemetry.addData("Lift Encoder", robot.getSpecificEncoderValue(0, false));
            //telemetry.update();

            setPower(power);
        }

        brake();

    }

    public void extend(double power) {
        //TODO: tune extension tic value later
        runToPosition(-1600, power); //was -1900
        extended = true;
    }

    //Thread(){
        //continue the I component of the loop (REMEMBER INTEGRAL WINDUP)
        //keep the thread running after your extension and retraction
    //}
    public void setPosPID(){
        //PID stuff
        //flag thread be active()
    }

    public void retract(double power){
        runToPosition(0, -power);
        extended = false;
    }

    public void brake(){
        setPower(0);
    }

}
