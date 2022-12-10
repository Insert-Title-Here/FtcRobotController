package org.firstinspires.ftc.teamcode.V2;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.League1.Common.Constants;
import org.firstinspires.ftc.teamcode.League1.Common.OpModeWrapper;
import org.firstinspires.ftc.teamcode.League1.Common.Vector2D;
import org.firstinspires.ftc.teamcode.League1.Subsystems.MecDrive;
import org.firstinspires.ftc.teamcode.League1.Subsystems.ScoringSystem2;
import org.firstinspires.ftc.teamcode.V2.NewSubsystem.ScoringSystemV2;
import org.firstinspires.ftc.teamcode.V2.NewSubsystem.ScoringSystemV2EpicLift;
import org.firstinspires.ftc.teamcode.V2.TeleOp.KevinGodModeV2;
import org.openftc.revextensions2.ExpansionHubMotor;

import java.io.FileNotFoundException;


@TeleOp
public class ArmTester extends OpModeWrapper {
    //Servo lServo, rServo, grabber;
    //DcMotor rMotor, lMotor;
    MecDrive drive;
    ScoringSystemV2EpicLift score;
    Constants constants;

    public ExpansionHubMotor rMotor1, lMotor1, rMotor2, lMotor2;

    Thread armThread, linkageThread;

    PassivePower passive;

    volatile boolean autoLinkageFlag, grabFlag, manualFlag, linkageUp, linkageDown;



    public enum PassivePower{
        //Feed forward is on
        EXTENDED,

        //Nothing
        MOVEMENT,

        //Power is set to 0
        ZERO,
    }


    @Override
    protected void onInitialize() throws FileNotFoundException {




        autoLinkageFlag = true;
        grabFlag = true;
        manualFlag = true;
        linkageDown = false;
        linkageUp = false;

        //Feed forward is going to be off
        passive = PassivePower.ZERO;

        //constants = new Constants();
        score = new ScoringSystemV2EpicLift(hardwareMap, constants);

        constants = new Constants();
        drive = new MecDrive(hardwareMap, false, telemetry);

        score.setGrabberPosition(constants.open - 0.07);


        armThread = new Thread(){
            @Override
            public void run() {

                while(opModeIsActive()){

                    //Lift up to scoring position
                    if(gamepad1.left_trigger > 0.1){

                        passive = PassivePower.MOVEMENT;
                        score.autoGoToPosition();
                        //score.setPower(0.2);
                        score.setLinkagePosition(constants.linkageScoreV2 - 0.02);
                        passive = PassivePower.EXTENDED;

                    }else {
                        if(passive == PassivePower.EXTENDED){
                            score.setPowerSingular(0.23);
                        }else if(passive == PassivePower.ZERO){
                            score.setPowerSingular(0);
                        }else if(passive == PassivePower.MOVEMENT){

                        }
                    }


                    //Scoring feature
                    if(gamepad1.right_trigger > 0.1){
                        score.setGrabberPosition(constants.score);

                        /*//Low height logic (need to lift slides up a bit before bringing linkage back for clearance)
                        if(score.getScoringMode() == ScoringSystemV2.ScoringMode.LOW && score.isExtended()) {
                            try {
                                sleep(500);
                            } catch (InterruptedException e) {

                            }
                            //passive = PassivePower.ZERO;
                            //score.moveToPosition(constants.lowOperation, 1);
                            //passive = PassivePower.EXTENDED;

                        }
*/
                        try {
                            sleep(600);
                        } catch (InterruptedException e) {
                            e.printStackTrace();
                        }


                        linkageDown = true;




                        //Do nothing during movement phase
                        //Reset to zero and no passive power
                        passive = PassivePower.MOVEMENT;

                        score.moveToPosition(0, 0.75);

                        //Open Grabber and reset linkage
                        score.setGrabberPosition(constants.open - 0.07);
                        //score.setLinkagePositionLogistic(constants.linkageDownV2, 300);
                        //score.setLinkagePositionLogistic(0.8, 500);

                        //TODO: fix this
                        /*score.lowerConeStack();
                        score.setLinkageConeStack(true);*/

                        //Resetting flags
                        autoLinkageFlag = true;
                        grabFlag = true;

                        //Not extended anymore
                        score.setExtended(false);

                        passive = PassivePower.ZERO;


                        //Automated Grab
                    }


                    //TODO: see if need to fix this logic
                    //Auto linkage up logic after sensing a cone



                    //Manual open and close grabber
                    if(gamepad1.start && manualFlag){
                        if(score.getGrabberPosition() != constants.open - 0.07) {
                            score.setGrabberPosition(constants.open - 0.07);
                            try {
                                sleep(300);
                            } catch (InterruptedException e) {
                                e.printStackTrace();
                            }
                            grabFlag = true;
                        }else{
                            score.setGrabberPosition(constants.grabbing);
                            try {
                                sleep(300);
                            } catch (InterruptedException e) {
                                e.printStackTrace();
                            }
                            grabFlag = false;
                        }
                        manualFlag = false;
                    }else if(!gamepad1.start){
                        manualFlag = true;
                    }



                    //Changing scoring modes (toggle)
                    if(gamepad1.y){
                        score.setScoringMode(ScoringSystemV2EpicLift.ScoringMode.LOW);

                    }else if(gamepad1.x){
                        score.setScoringMode(ScoringSystemV2EpicLift.ScoringMode.MEDIUM);

                    }else if(gamepad1.b){
                        score.setScoringMode(ScoringSystemV2EpicLift.ScoringMode.HIGH);

                    }

                    //Manual slides (dpad right and left)
                    if(gamepad1.dpad_right){
                        //passive = PassivePower.MOVEMENT;
                        score.setPower(0.5);
                    }else if(gamepad1.dpad_left){
                        //passive = PassivePower.MOVEMENT;
                        score.setPower(-0.5);
                    }else{

                        //Feedforward if slides are extended
                        if(score.isExtended() == true){
                            passive = PassivePower.EXTENDED;
                        }else{
                            passive = PassivePower.ZERO;
                        }


                    }


                }

            }
        };


        linkageThread = new Thread() {

            @Override
            public void run() {
                while(opModeIsActive()) {
                    if(linkageUp) {

                        try {
                            Thread.sleep(100);
                        } catch (InterruptedException e) {
                            e.printStackTrace();
                        }
                        score.setLinkagePosition(Constants.linkageScoreV2 - 0.02);
                        linkageUp = false;
                    }else if(linkageDown) {

                        score.setLinkagePosition(Constants.linkageUpV2);
                        try {
                            Thread.sleep(70);
                        } catch (InterruptedException e) {
                            e.printStackTrace();
                        }
                        score.setLinkagePositionLogistic(Constants.linkageDownV2, 220);
                        linkageDown = false;
                    }
                }
            }
        };





    }

    @Override
    protected void onStart() {
        armThread.start();
        linkageThread.start();

        score.setLinkagePosition(Constants.linkageDownV2);



        while(opModeIsActive()) {
            double yPos = getYCapPosition();
            setYCapPosition(yPos - map(gamepad2.right_stick_y, -1, 1, -0.0010, 0.0010));

            telemetry.addData("Position", getYCapPosition());
            telemetry.update();

        }




        
    }

    @Override
    protected void onStop() {
        score.setPower(0);
    }


    public void setYCapPosition(double pos){
        score.setLinkagePosition(pos);
    }

    public double map(double val, double in_min, double in_max, double out_min, double out_max) {
        return (val - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
    }


    public double getYCapPosition() {
        return score.getLeftLinkage();
    }

}
