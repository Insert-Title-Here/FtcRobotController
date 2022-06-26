package teamcode.Competition.Subsystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;


import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.openftc.revextensions2.ExpansionHubMotor;

import teamcode.common.AbstractOpMode;
import teamcode.common.Debug;
import teamcode.common.RobotPositionStateUpdater;
import teamcode.common.Utils;
public class ArmSystem {

    //House Servo values
    private static final double INTAKE_POSITION = 0.12; //0
    private static final double HOUSING_POSITION_BALL = 0.19;
    private static final double HOUSING_POSITION_DUCK = 0.26; //0.12
    private static final double HOUSING_POSITION = 0.20; //0.22
    private static final double SCORING_POSITION = 0.60; //0.5
    private static final double SCORING_POSITION_CONVEYOR = 0.60;

    private static final double LINKAGE_DOWN = 0.3; //these values need to be refined but they are good ballparks. AYUSH: No longer a final constant.
    private static final double LINKAGE_HOUSED = 0.58;
    private static final double LINKAGE_SCORE = 0.7;

    private static final double WINCHSTOP_STOPPING = 0.45;
    private static final double WINCHSTOP_INIT = 0.6;
    private static final double WINCHSTOP_OPEN = 1.0;


    private static final float GREEN_THRESHOLD = 255; //not needed for now
    private static final float RED_THRESHOLD = 255;
    private static final float BLUE_THRESHOLD = 255;
    private static final int YELLOW_THRESHOLD = 02552550;
    private static final int WHITE_THRESHOLD = 0255255255;

    private static final double SLIDE_POWER = 1.0;
    private static final long TIMEOUT_MILLIS = 5000;

    private ExpansionHubMotor winchMotor, winchEncoder, conveyorMotor;
    private DcMotorEx intake;
    private Servo house, linkage, rampWinch, winchStop;
    RobotPositionStateUpdater.RobotPositionState currentState;
    private Stage stage;
    private boolean isDuck;
    private boolean isTeleOp;


    public ArmSystem(HardwareMap hardwareMap, boolean isTeleOp){
        intake = hardwareMap.get(DcMotorEx.class,"Intake");
        winchMotor = (ExpansionHubMotor) hardwareMap.dcMotor.get("Winch");
        winchEncoder = (ExpansionHubMotor) hardwareMap.dcMotor.get("WinchEncoder"); //TANK FrontLeftDrive MECANUM Winch
        conveyorMotor = (ExpansionHubMotor) hardwareMap.dcMotor.get("Ramp");

        house = hardwareMap.servo.get("House");
        linkage = hardwareMap.servo.get("Linkage");
        //rampWinch = hardwareMap.servo.get("RampWinch");
        winchStop = hardwareMap.servo.get("Stop");
        winchStop.setDirection(Servo.Direction.REVERSE);
        //carousel = hardwareMap.get(CRServo.class, "Carousel");

        winchMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        winchEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        winchEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        winchEncoder.setDirection(DcMotorSimple.Direction.REVERSE);
        //carousel.setDirection(DcMotorSimple.Direction.REVERSE);

        winchStop.setPosition(WINCHSTOP_INIT);

        if(isTeleOp){
            house.setPosition(INTAKE_POSITION);
        }else{
            house.setPosition(HOUSING_POSITION);
        }
        linkage.setPosition(LINKAGE_HOUSED - 0.1);
        stage = Stage.IDLE;
        isDuck = false;
        this.isTeleOp = isTeleOp;
    }

    public void intakeDumb(double power){
        intake.setPower(power);

    }
    public void setSlidePower(double power){
        winchMotor.setPower(power);
    }

    public void actuateWinchStop(double pos){
        winchStop.setPosition(pos);
    }

    public int getConveyorPosition(){
        return conveyorMotor.getCurrentPosition();
    }

    public void intakeReverseTest(){

        double previousTics = conveyorMotor.getCurrentPosition();
        double deltaTics = 10;
        double startTime = AbstractOpMode.currentOpMode().time;
        double deltaTime = AbstractOpMode.currentOpMode().time - startTime;
        conveyorMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        conveyorMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeDumb(1.0);
        boolean dticsFlag = true;
        while(AbstractOpMode.currentOpMode().opModeIsActive() && !AbstractOpMode.currentOpMode().isStopRequested()){
            int currentTics = conveyorMotor.getCurrentPosition();
            deltaTics = currentTics - previousTics;
            previousTics = currentTics;
            deltaTime = AbstractOpMode.currentOpMode().time - startTime;
            if(deltaTics < 1 &&  deltaTime > 1.0){
                Utils.sleep(500);
                if(deltaTics < 1){
                    intakeDumb(-1.0);
                    Utils.sleep(250);
                    intakeDumb(1.0);
                    startTime = AbstractOpMode.currentOpMode().time;
                }
            }

            AbstractOpMode.currentOpMode().telemetry.addData("delta", deltaTics);
            AbstractOpMode.currentOpMode().telemetry.addData("time", deltaTime);
            AbstractOpMode.currentOpMode().telemetry.update();
        }


    }





    public double getMilliAmps(){
        return intake.getCurrent(CurrentUnit.MILLIAMPS);
    }

    public double getMilliAmpsWinch(){
        return winchMotor.getCurrent(CurrentUnit.MILLIAMPS);
    }





    public void runConveyor(double power){
        if(power != 0) {
            //linkage.setPosition(LINKAGE_SCORE + 0.2);
            house.setPosition(SCORING_POSITION_CONVEYOR - 0.12);
        }
        conveyorMotor.setPower(power);
    }

    public void runConveyorPos(double power, int position){
        //linkage.setPosition(LINKAGE_SCORE);
        house.setPosition(SCORING_POSITION_CONVEYOR - 0.1);
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
    private final double LOW_POS = 0;
    private final double RETRACTED_POS = 0.2;
    private final double TIPPING_POS = 0.15;
    public void setRampWinchLow(){
        rampWinch.setPosition(LOW_POS);
    }
    public void setRampWinchTipped(){
        rampWinch.setPosition(TIPPING_POS);
    }

    public void setRampWinchRetracted(){
        rampWinch.setPosition(RETRACTED_POS);
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
        actuateWinchStop(WINCHSTOP_OPEN);
        if(isTeleOp && !isDuck) {
            intakeDumb(1.0);
        }
        linkage.setPosition(LINKAGE_HOUSED);
        Utils.sleep(250);
        synchronized (this) {
            if (isDuck) {
                house.setPosition(HOUSING_POSITION_DUCK);
            } else {
                //house.setPosition(HOUSING_POSITION);
            }
        }
        intakeDumb(-1);
        setHouse();
        Utils.sleep(550);
        intakeDumb(0);
        stage = Stage.HOUSED;

        //Debug.log("finish");
    }

    public void duckPreScore() {
        actuateWinchStop(WINCHSTOP_OPEN);
        if (isTeleOp && !isDuck) {
            intakeDumb(1.0);
        }
        linkage.setPosition(LINKAGE_HOUSED);
        Utils.sleep(250);
        synchronized (this) {
            if (isDuck) {
                house.setPosition(HOUSING_POSITION_DUCK);
            } else {
                // house.setPosition(HOUSING_POSITION);
            }
        }
        setHouse();
        Utils.sleep(550);
        intakeDumb(0);
        stage = Stage.HOUSED;
    }

    public void setHouse() {
        house.setPosition(HOUSING_POSITION_DUCK);
    }

    public void preScoreMultiFreight(){
        actuateWinchStop(WINCHSTOP_OPEN);


        //house.setPosition(HOUSING_POSITION);
        Utils.sleep(250);
        linkage.setPosition(LINKAGE_HOUSED);
        house.setPosition(0.08 ); // 0.12
    }
    public void preScoreAuto(){
        if(isDuck){
            house.setPosition(HOUSING_POSITION_DUCK);
        }else {
            house.setPosition(HOUSING_POSITION);
        }
        Utils.sleep(550);
        linkage.setPosition(LINKAGE_HOUSED);
        intakeDumb(-1.0);

        stage = Stage.HOUSED;

        //Debug.log("finish");


    }

    @Deprecated
    public void preScoreDuck(){
        house.setPosition(HOUSING_POSITION +0.12);
        Utils.sleep(500);
        linkage.setPosition(LINKAGE_HOUSED);
        stage = Stage.HOUSED;
    }

    public void jitterHouse(){
        synchronized (this){
            house.setPosition(HOUSING_POSITION_BALL);
        }
        Utils.sleep(250);
        house.setPosition(SCORING_POSITION);
    }



    public boolean isLinkageInPreScore(){
        return linkage.getPosition() != LINKAGE_SCORE;
    }



    public void raise(double position) {



        house.setPosition(HOUSING_POSITION);
        moveSlide(SLIDE_POWER, (int) position);

        //Utils.sleep(200);

        if(!isTeleOp){
            linkage.setPosition(LINKAGE_SCORE);
        }

        linkage.setPosition(LINKAGE_SCORE);

        stage = stage.EXTENDED;
    }


    public void setLinkageScored(){
        linkage.setPosition(LINKAGE_SCORE);
    }

    public void sharedRaise(double position) {

        house.setPosition(HOUSING_POSITION);
        Utils.sleep(200);
        moveSlide(SLIDE_POWER, (int) position);
        stage = stage.EXTENDED;
    }

    //temporary tele op scoring function w/o color sensor
    public synchronized void score(){
        house.setPosition(SCORING_POSITION);
    }
    public synchronized void scoreAuto(){
        house.setPosition(SCORING_POSITION+0.05);
    }

    public synchronized void scoreAuto(boolean far){
        if(far){
            house.setPosition(SCORING_POSITION + 0.2);
        }else{
            house.setPosition(SCORING_POSITION);
        }
    }

    public synchronized void retract(){
        linkage.setPosition(LINKAGE_HOUSED);
        while (winchEncoder.getCurrentPosition()  > 1000 && AbstractOpMode.currentOpMode().opModeIsActive() && !AbstractOpMode.currentOpMode().isStopRequested()) {
//                AbstractOpMode.currentOpMode().telemetry.addData("curR", winchEncoder.getCurrentPosition());
//                AbstractOpMode.currentOpMode().telemetry.addData("tarR", position);
//                AbstractOpMode.currentOpMode().telemetry.update();
            winchMotor.setPower(-0.5);
        }
        winchMotor.setPower(0);
       idleServos();
    }
    public synchronized void retractZero(){
        linkage.setPosition(LINKAGE_HOUSED);
        while (winchEncoder.getCurrentPosition()  > 0 && AbstractOpMode.currentOpMode().opModeIsActive() && !AbstractOpMode.currentOpMode().isStopRequested()) {
//                AbstractOpMode.currentOpMode().telemetry.addData("curR", winchEncoder.getCurrentPosition());
//                AbstractOpMode.currentOpMode().telemetry.addData("tarR", position);
//                AbstractOpMode.currentOpMode().telemetry.update();
            winchMotor.setPower(-0.3);
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

        actuateWinchStop(WINCHSTOP_STOPPING);
        Utils.sleep(100);
        if (isDuck) {
            house.setPosition(INTAKE_POSITION);
        } else{
            house.setPosition(INTAKE_POSITION);
        }
        linkage.setPosition(LINKAGE_DOWN);
    }

    public void lowerLinkageAuto() {
        actuateWinchStop(WINCHSTOP_STOPPING);
        synchronized (this) {
            linkage.setPosition(LINKAGE_DOWN);
        }
        house.setPosition(INTAKE_POSITION ); //+0.156

    }

    public void setIsDuck(boolean isDuck) {
        this.isDuck = isDuck;
    }

    public void setConveyorMode(DcMotor.RunMode mode) {
        conveyorMotor.setMode(mode);
    }

    public void scoreFar() {

        house.setPosition(0.9);
    }

    public enum Stage{
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
            while (Math.abs(winchEncoder.getCurrentPosition()) < position && AbstractOpMode.currentOpMode().opModeIsActive() && !AbstractOpMode.currentOpMode().isStopRequested()) {
                AbstractOpMode.currentOpMode().telemetry.addData("curE", winchEncoder.getCurrentPosition());
                AbstractOpMode.currentOpMode().telemetry.addData("tarE",position);
                AbstractOpMode.currentOpMode().telemetry.update();
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

    public void sinIntake(double min, double max, double dtMax){
        double start = AbstractOpMode.currentOpMode().time;
        double dt = AbstractOpMode.currentOpMode().time;
        while(dt < dtMax){
            dt = AbstractOpMode.currentOpMode().time - start;
            double amplitude = max - min;
            intakeDumb(amplitude * Math.sin(dt) + (amplitude / 2.0) + min);
        }
    }
    boolean terminateIntake;
    public void sinIntakeIndefinite(double min, double max){
        lowerLinkage();
        double start = AbstractOpMode.currentOpMode().time;
        double dt = AbstractOpMode.currentOpMode().time;
        terminateIntake = false;
        while(!terminateIntake && !AbstractOpMode.currentOpMode().isStopRequested() && AbstractOpMode.currentOpMode().opModeIsActive()){
            dt = AbstractOpMode.currentOpMode().time - start;
            double amplitude = max - min;
            intakeDumb(amplitude * Math.abs(Math.sin(5 * dt))+ min);
        }
        intakeDumb(0);
    }

    public void setTerminateIntake(boolean val){
        this.terminateIntake = val;
    }

}
