package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

//This is a experiment to see if i can acutally make our robot move
//by DragonStryke

@Autonomous(name = "TestAutoOpModeC", group = "LinearOpmode")
@Disabled

public class testAutoOpModeC extends LinearOpMode {
    //opmode members
    private ElapsedTime  runtime = new ElapsedTime();
    private DcMotor LbDrive = null;
    private DcMotor RbDrive = null;
    private DcMotor LfDrive = null;
    private DcMotor RfDrive = null;



    @Override
    public void runOpmode() {
        //telemetry data to signal robot waiting
        telemetry.addData("Status", "Ready to run");    //
        telemetry.update();

        //initiallizing hardware variables
        LfDrive  = hardwareMap.get(DcMotor.class, "left_front_drive");
        RfDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        LbDrive = hardwareMap.get(DcMotor.class, "left_back_drive");
        RbDrive = hardwareMap.get(DcMotor.class, "right_back_drive");




    }
}
