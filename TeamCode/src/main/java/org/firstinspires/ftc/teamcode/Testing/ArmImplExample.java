package org.firstinspires.ftc.teamcode.Testing;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Robot;

public class ArmImplExample {


    private static final int LIFT_TOLERANCE = 20; //arbetrary
    private DcMotor lift;
    private Robot robot;
    private boolean isArmOnChub = false;

    public ArmImplExample(HardwareMap hardwareMap, Robot robot){
        lift = hardwareMap.dcMotor.get("lift");
        this.robot = robot;
    }


    public void move(int tics, double power){
        LynxModule.BulkData data = robot.getBulkPacket(isArmOnChub);
        while(Math.abs(data.getMotorCurrentPosition(0) - tics) < LIFT_TOLERANCE){
            lift.setPower(power);
            data = robot.getBulkPacket(isArmOnChub);
        }

    }


}
