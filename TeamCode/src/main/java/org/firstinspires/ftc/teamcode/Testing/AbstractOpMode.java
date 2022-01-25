package org.firstinspires.ftc.teamcode.Testing;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


public abstract class AbstractOpMode extends LinearOpMode {
    private static AbstractOpMode opMode;
    public static AbstractOpMode currentOpMode() {
        return opMode;
    }

    @Override
    public final void runOpMode(){
        opMode = this;
        Debug.clear();
        try{
            onInitialize();
            waitForStart();
            onStart();
            onStop();
        }catch(Exception e){
            Debug.log(e);
            Utils.sleep(5000);
        }
    }

    protected abstract void onInitialize();

    protected abstract void onStart();

    protected abstract void onStop();

}
