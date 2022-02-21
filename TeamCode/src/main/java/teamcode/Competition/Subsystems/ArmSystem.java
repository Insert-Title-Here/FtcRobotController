package teamcode.Competition.Subsystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;


import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.openftc.revextensions2.ExpansionHubMotor;

import teamcode.common.AbstractOpMode;
import teamcode.common.Debug;
import teamcode.common.RobotPositionStateUpdater;
import teamcode.common.Utils;
public class ArmSystem {

    //House Servo values
    private static final double INTAKE_POSITION = 0.0;
    private static final double HOUSING_POSITION = 0.12 ; //these values are great, the scoring one MAYBE move up a lil but no more than 0.66 because it grinds at that point
    private static final double SCORING_POSITION = 0.5;

    private static final double LINKAGE_DOWN = 0.26; //these values need to be refined but they are good ballparks. AYUSH: No longer a final constant.
    private static final double LINKAGE_HOUSED = 0.6;
    private static final double LINKAGE_SCORE = 0.8;


    private static final float GREEN_THRESHOLD = 255; //not needed for now
    private static final float RED_THRESHOLD = 255;
    private static final float BLUE_THRESHOLD = 255;
    private static final int YELLOW_THRESHOLD = 02552550;
    private static final int WHITE_THRESHOLD = 0255255255;

    private static final double SLIDE_POWER = 1.0;
    private static final long TIMEOUT_MILLIS = 5000;

    private ExpansionHubMotor intake, winchMotor, winchEncoder, conveyorMotor;
    private Servo house, linkage;
    RobotPositionStateUpdater.RobotPositionState currentState;
    private Stage stage;


    public ArmSystem(HardwareMap hardwareMap, boolean isTeleOp){
        intake = (ExpansionHubMotor) hardwareMap.dcMotor.get("Intake");
        winchMotor = (ExpansionHubMotor) hardwareMap.dcMotor.get("Winch");
        winchEncoder = (ExpansionHubMotor) hardwareMap.dcMotor.get("WinchEncoder"); //TANK FrontLeftDrive MECANUM Winch
        conveyorMotor = (ExpansionHubMotor) hardwareMap.dcMotor.get("Ramp");

        house = hardwareMap.servo.get("House");
        linkage = hardwareMap.servo.get("Linkage");
        //carousel = hardwareMap.get(CRServo.class, "Carousel");

        winchMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        winchEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        winchEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        winchEncoder.setDirection(DcMotorSimple.Direction.REVERSE);
        //carousel.setDirection(DcMotorSimple.Direction.REVERSE);

        if(isTeleOp){
            house.setPosition(INTAKE_POSITION);
        }else{
            house.setPosition(HOUSING_POSITION);
        }
        linkage.setPosition(LINKAGE_HOUSED);
        stage = Stage.IDLE;
    }

    public void intakeDumb(double power){
        intake.setPower(power);

    }






    public void runConveyor(double power){
        conveyorMotor.setPower(power);
    }

    public void runConveyorPos(double power, int position){
        linkage.setPosition(LINKAGE_SCORE);
        house.setPosition(SCORING_POSITION);
        conveyorMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        conveyorMotor.setTargetPosition(position);
        conveyorMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while(Math.abs(conveyorMotor.getCurrentPosition() - conveyorMotor.getTargetPosition()) > 10){


            conveyorMotor.setPower(power);
        }
        conveyorMotor.setPower(0);
    }


    public void intake(double intakePower, boolean isAuto){
        if(isAuto){
            lowerLinkage();
        }

        if(stage == stage.HOUSED){
            stage = stage.INTAKING;
        }

        if(stage == stage.INTAKING || stage == Stage.IDLE) {
            boolean detectedElement = false;

            intakeDumb(intakePower);
            stage = Stage.INTAKING;
            if(detectedElement) {

                preScore();
            }
        }
    }

    public boolean intakeAuto(double intakePower){
        lowerLinkage();
        Utils.sleep(400);
        boolean detectedElement = false;
        long start = System.currentTimeMillis();
        while(!detectedElement){
            long current = System.currentTimeMillis();

            intakeDumb(intakePower);
            if(detectedElement) {
                intakeDumb(-0.9);
                Utils.sleep(500);
                preScore();
                intakeDumb(0);
                return true;
            }else if(current - start > TIMEOUT_MILLIS){
                intakeDumb(0);
                return false;
            }
        }
        return false;

    }

    //will be merged into intake() later
    public void preScore(){
        house.setPosition(HOUSING_POSITION);
        Utils.sleep(250);
        intakeDumb(-1.0);
        linkage.setPosition(LINKAGE_HOUSED);
        stage = Stage.HOUSED;

        //Debug.log("finish");


    }
    public void preScoreAuto(){
        house.setPosition(HOUSING_POSITION);
        Utils.sleep(250);
        linkage.setPosition(LINKAGE_HOUSED);
        intakeDumb(-1.0);

        stage = Stage.HOUSED;

        //Debug.log("finish");


    }

    public void preScoreDuck(){
        house.setPosition(HOUSING_POSITION + 0.02 );
        Utils.sleep(250);
        linkage.setPosition(LINKAGE_HOUSED);
        stage = Stage.HOUSED;
    }

    public boolean isLinkageInPreScore(){
        return linkage.getPosition() != LINKAGE_SCORE;
    }



    public synchronized void raise(double position) {

        linkage.setPosition(LINKAGE_SCORE);
        moveSlide(SLIDE_POWER, (int) position);

        stage = stage.EXTENDED;
    }

    //temporary tele op scoring function w/o color sensor
    public synchronized void score(){
        house.setPosition(SCORING_POSITION);
    }

    public synchronized void scoreAuto(boolean far){
        if(far){
            house.setPosition(SCORING_POSITION + 0.2);
        }else{
            house.setPosition(SCORING_POSITION);
        }
    }

    public synchronized void retract(){
        while (winchEncoder.getCurrentPosition()  > 0) {
//                AbstractOpMode.currentOpMode().telemetry.addData("curR", winchEncoder.getCurrentPosition());
//                AbstractOpMode.currentOpMode().telemetry.addData("tarR", position);
//                AbstractOpMode.currentOpMode().telemetry.update();
            winchMotor.setPower(-1.0);
        }
        winchMotor.setPower(0);
       idleServos();
    }

    public void idleServos(){
        house.setPosition(INTAKE_POSITION);
        linkage.setPosition(LINKAGE_HOUSED);
        stage = Stage.IDLE;
    }


    public void setWinchPower(double v) {
        winchMotor.setPower(v);
    }

    public void setWinchVelocity(double v, AngleUnit angleUnit){
        winchMotor.setVelocity(v, angleUnit);
    }

    public void lowerLinkage() {
        house.setPosition(INTAKE_POSITION);
        linkage.setPosition(LINKAGE_DOWN);
    }

    private enum Stage{
        INTAKING, IDLE, HOUSED, EXTENDED
    }

    //tele op scoring function, assumes the freight is encapsulated in the house already and that the
    //linkage is raised (not scoring). This method also assumes the Conveyor exists as well so if we
    //get rid of the conveyor we need to change this
    //uses color sensor data
//    public void scoreCS(){
//        if(stage == Stage.CUBE_HOUSED) {
//
//        }else if(stage == Stage.BALL_HOUSED){
//            linkage.setPosition(LINKAGE_SCORE);
//            moveSlide(SLIDE_POWER, TOP_POSITION);
//            Utils.sleep(200);
//            house.setPosition(SCORING_POSITION);
//            Utils.sleep(500);
//            moveSlide(-SLIDE_POWER, BOTTOM_POSITION);
//
//        }
//        house.setPosition(INTAKE_POSITION);
//        linkage.setPosition(LINKAGE_DOWN);
//        stage = Stage.IDLE;

    //}

    public void moveSlide(double power, double position){
            while (winchEncoder.getCurrentPosition() < position) {
//                AbstractOpMode.currentOpMode().telemetry.addData("curE", winchEncoder.getCurrentPosition());
//                AbstractOpMode.currentOpMode().telemetry.addData("tarE",position);
//                AbstractOpMode.currentOpMode().telemetry.update();
                winchMotor.setPower(power);
            }
            //winchMotor.setVelocity(FEEDFORWARD_V, AngleUnit.RADIANS);
            winchMotor.setPower(0);

    }

    public void moveSlideAuto(double power, double position){
        while (winchEncoder.getCurrentPosition() < position) {
//                AbstractOpMode.currentOpMode().telemetry.addData("curE", winchEncoder.getCurrentPosition());
//                AbstractOpMode.currentOpMode().telemetry.addData("tarE",position);
//                AbstractOpMode.currentOpMode().telemetry.update();
            winchMotor.setPower(power);
        }
        winchMotor.setVelocity(FEEDFORWARD_V, AngleUnit.RADIANS);


    }

    public double getSign(double num){
        if(num >= 0){
            return 1.0;
        }else{
            return -1.0;
        }
    }
    public final double FEEDFORWARD_V = 0.01; //3.0156 * 0.0254
    public void moveSlideNew(double power, int position){
        winchEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        AbstractOpMode.currentOpMode().telemetry.addData("current pos ", winchEncoder.getCurrentPosition());
        AbstractOpMode.currentOpMode().telemetry.update();

        winchEncoder.setTargetPosition(position);
        winchEncoder.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        winchMotor.setPower(-power);
        while(winchEncoder.getCurrentPosition() < winchEncoder.getTargetPosition()){
//            AbstractOpMode.currentOpMode().telemetry.addData("current pos ", winchEncoder.getCurrentPosition());
//            AbstractOpMode.currentOpMode().telemetry.addData("target pos ", winchEncoder.getTargetPosition());
//            AbstractOpMode.currentOpMode().telemetry.update();
        }
        Debug.log("LOOP TERMINATED.");
        winchMotor.setVelocity(FEEDFORWARD_V, AngleUnit.RADIANS);
    }

    //fix the winch shooting off into space

    public void resetWinchEncoder(){
        winchEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        winchEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }



    public int getLinearSlidePosition(){
        return winchEncoder.getCurrentPosition();
    }


    public Stage getStage(){
        return stage;
    }

}
