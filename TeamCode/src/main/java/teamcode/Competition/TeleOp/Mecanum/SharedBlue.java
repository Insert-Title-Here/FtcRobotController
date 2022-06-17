package teamcode.Competition.TeleOp.Mecanum;


import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import teamcode.Competition.Subsystems.ArmSystem;
import teamcode.Competition.Subsystems.EndgameSystems;
import teamcode.common.AbstractOpMode;
import teamcode.common.Constants;
import teamcode.common.Debug;
import teamcode.common.Localizer;
import teamcode.common.MecanumDriveTrain;
import teamcode.common.Utils;
import teamcode.common.Vector2D;

@TeleOp(name="Shared Blue")
public class SharedBlue extends AbstractOpMode {
    MecanumDriveTrain drive;
    ArmSystem arm;
    Thread armThread, capThread;
    EndgameSystems systems;

    private ScoredButtonState state;
    private PulleyState pulleyState;
    private LinkageState linkageState;
    private long scoredSampleTime;
    private boolean isDuck;

    @Override
    protected void onInitialize() {
        drive = new MecanumDriveTrain(hardwareMap);
        arm = new ArmSystem(hardwareMap, true);
        systems = new EndgameSystems(hardwareMap, true);

        state = ScoredButtonState.RETRACTING;
        pulleyState = PulleyState.RETRACTED;
        linkageState = linkageState.RAISED;


        armThread = new Thread(){
            public void run(){
                while(opModeIsActive()){
                    armUpdate();
                }
            }
        };

        capThread = new Thread() {
            public void run() {
                while (opModeIsActive()) {
                    capUpdate();
                }
            }
        };

        systems.zeroCap();
        previousLeft = false;
        previousRight = false;
        previousUp = false;
        previousDown = false;
        previousStart = false;
        previousExtensionTime = 0;
        iterator = 0;
        isExtended = false;
        isDuck = false;
        arm.setIsDuck(isDuck);
    }

    // Flag variable for keeping every servo frozen until game start
    boolean capping = false;
    private double feedPow = 0.05;
    boolean previousLeft, previousRight, previousUp, previousDown;
    private void capUpdate() {
        if(gamepad2.right_trigger > 0 || gamepad2.left_trigger > 0) {
            double val = gamepad2.right_trigger - gamepad2.left_trigger;
            systems.setCapstoneExtensionPower(val);
        } else if (gamepad2.right_bumper) {
            systems.setCapstoneExtensionPower(0.5);
        } else if (gamepad2.left_bumper) {
            systems.setCapstoneExtensionPower(-0.5);
        } else{
            systems.setCapstoneExtensionPower(0);
        }


//        telemetry.addData("feedpow", feedPow);
//        telemetry.update();


        double yPos = systems.getYCapPosition();
        systems.setXCapstoneRotatePower(gamepad2.left_stick_x);

//        telemetry.addData("rsy", gamepad2.right_stick_y);
//        telemetry.update();
        systems.setYCapPosition(yPos - systems.map(gamepad2.right_stick_y, -1, 1, -0.0010, 0.0010));

        if (gamepad2.x) {
            systems.zeroCap();
        } else if (gamepad2.dpad_right && previousRight != gamepad2.dpad_right) {
            systems.setXCapSpeedDivisor(7);
        } else if (gamepad2.dpad_left && previousLeft != gamepad2.dpad_left) {
            systems.setXCapSpeedDivisor(5);
        }

        previousLeft = gamepad2.dpad_left;
        previousRight = gamepad2.dpad_right;
        previousUp = gamepad2.dpad_up;
        previousDown = gamepad2.dpad_down;
    }

    double startTime;
    double previousExtensionTime;
    int iterator;
    boolean isExtended, previousStart, previousOptions;
    volatile boolean isEndgame = false;
    private void armUpdate() {
        if (gamepad1.right_trigger > 0.3) {
            startTime = AbstractOpMode.currentOpMode().time;
            while (gamepad1.right_trigger > 0.3) {
                double elapsedTime = AbstractOpMode.currentOpMode().time - startTime;
                if (elapsedTime < 0.5 && linkageState == linkageState.RAISED) {
                    arm.lowerLinkage();
                    linkageState = LinkageState.LOWERED;
                } else {
                    if(isDuck){
                        arm.intakeDumb(0.8);
                    }else{
                        arm.intakeDumb(1.0);
                    }
                    //arm.intakeDumb(0.3 * Math.abs(Math.sin(6 * elapsedTime)) + 0.7);
                }

            }
        } else if (gamepad1.a) {

            //add something to move the linkage outta the way
            while (gamepad1.a) {
                arm.intakeDumb(-1.0);
            }
        } else if (gamepad1.left_stick_button) {
            arm.runConveyor(0.8);
            arm.retract();
        } else if (gamepad1.dpad_up) {
            // set to full
            arm.setWinchPower(1);
        } else if (gamepad1.dpad_down) {
            arm.setWinchPower(-0.5);
        } else if (gamepad1.left_trigger > 0.3) {
            if (pulleyState == PulleyState.RETRACTED && linkageState == LinkageState.RAISED) {
                previousExtensionTime = time;
                if(isExtended) {
                    arm.raise(Constants.TOP_POSITION + 1000); //3000
                }else{
                    arm.raise(Constants.TOP_POSITION);
                }
                pulleyState = PulleyState.HIGH_GOAL;
                linkageState = LinkageState.RAISED;
            }
        } else if (gamepad1.dpad_right && pulleyState == PulleyState.RETRACTED) {
//            if (pulleyState == PulleyState.RETRACTED && linkageState == LinkageState.RAISED && time - previousExtensionTime > 5) {
//                if(isExtended){
//                    arm.raise(Constants.MEDIUM_POSITION + 2000);
//                }else {
//                    arm.raise(Constants.MEDIUM_POSITION);
//                }
//                pulleyState = PulleyState.MID_GOAL;
//                linkageState = linkageState.RAISED;
//            }
        } else if (gamepad1.y && pulleyState == PulleyState.RETRACTED) {
            //arm.raise(Constants.BOTTOM_POSITION);
            while(gamepad1.y){
                systems.runCarousel(-1.0);
            }
            //arm.runConveyorPos(0.5, 2000);
            arm.idleServos();
            //arm.moveSlide(-1, 0);
            pulleyState = PulleyState.RETRACTED;
        } else if (gamepad1.dpad_left) {
            arm.resetWinchEncoder();
        }else if(gamepad1.right_bumper){
            arm.jitterHouse();
        }
        if (gamepad1.left_bumper) {
            while (gamepad1.left_bumper) {
                arm.runConveyor(0.9);
            }
            arm.retract();
            arm.idleServos();
        } else if (gamepad1.start && !previousStart){
            isExtended = !isExtended;
        }else if(gamepad1.square && !previousOptions){
            isDuck = !isDuck;
            isExtended = isDuck;
            arm.setIsDuck(isDuck);

        }else if(gamepad1.b){
            while(gamepad1.b){
                systems.runCarousel(-0.2);
            }
        } else{
            arm.setWinchPower(0);
            systems.runCarousel(0);
            arm.intakeDumb(0);
            arm.runConveyor(0);
        }
        if (gamepad1.right_stick_button) {
            arm.preScore();
            arm.intakeDumb(-1.0);
            linkageState = LinkageState.RAISED;
            Utils.sleep(150);
            arm.sharedRaise(3000);
            arm.intakeDumb(0);
        }
        previousOptions = gamepad1.square;
        previousStart = gamepad1.start;
        telemetry.addData("isExtended", isExtended);
        telemetry.addData("slide pos", arm.getLinearSlidePosition());
        telemetry.addData("isDuck", isDuck);
        telemetry.update();
    }

    @Override
    protected void onStart() {
        armThread.start();
        capThread.start();
        while(opModeIsActive()){

            drive.setPower(new Vector2D(-gamepad1.left_stick_y, gamepad1.left_stick_x),  0.7 * gamepad1.right_stick_x);
        }
    }

    private enum ScoredButtonState{
        SCORED, RETRACTING
    }

    private enum PulleyState{
        HIGH_GOAL, MID_GOAL, RETRACTED
    }

    private enum LinkageState{
        RAISED, LOWERED
    }

    private enum CycleStyle{
        HIGH, MID
    }


    @Override
    protected void onStop() {
        //armThread.interrupt();
        //capThread.interrupt();
    }
}
