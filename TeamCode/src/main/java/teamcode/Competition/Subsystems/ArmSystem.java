package teamcode.Competition.Subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;


import teamcode.common.AbstractOpMode;
import teamcode.common.Debug;
import teamcode.common.RobotPositionStateUpdater;
import teamcode.common.Utils;

public class ArmSystem {

    //House Servo values
    private static final double INTAKE_POSITION = 0.0;
    private static final double HOUSING_POSITION = 0.2; //these values are great, the scoring one MAYBE move up a lil but no more than 0.66 because it grinds at that point
    private static final double SCORING_POSITION = 0.37;

    private static final double LINKAGE_DOWN = 0.0; //these values need to be refined but they are good ballparks. AYUSH: No longer a final constant.
    private static final double LINKAGE_HOUSED = 0.7;
    private static final double LINKAGE_SCORE = 0.7;


    private static final float GREEN_THRESHOLD = 255; //not needed for now
    private static final float RED_THRESHOLD = 255;
    private static final float BLUE_THRESHOLD = 255;
    private static final int YELLOW_THRESHOLD = 02552550;
    private static final int WHITE_THRESHOLD = 0255255255;

    private static final double SLIDE_POWER = 1.0;
    private static final long TIMEOUT_MILLIS = 5000;

    private DcMotor intake, winchMotor, winchEncoder, conveyorMotor;
    private Servo house, linkage;
    private NormalizedColorSensor sensor;
    RobotPositionStateUpdater.RobotPositionState currentState;
    private Stage stage;


    public ArmSystem(HardwareMap hardwareMap, boolean isTeleOp){
        intake = hardwareMap.dcMotor.get("Intake");
        winchMotor = hardwareMap.dcMotor.get("Winch");
        winchEncoder = hardwareMap.dcMotor.get("WinchEncoder"); //TANK FrontLeftDrive MECANUM Winch
        //conveyorMotor = hardwareMap.dcMotor.get("Conveyor");

        house = hardwareMap.servo.get("House");
        linkage = hardwareMap.servo.get("Linkage");
        //carousel = hardwareMap.get(CRServo.class, "Carousel");

        sensor = hardwareMap.get(NormalizedColorSensor.class, "color");
        sensor.setGain(280); //325 is tested value but i think I trust this one more
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
        house.setPosition(SCORING_POSITION);
        conveyorMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        conveyorMotor.setTargetPosition(-position);
        conveyorMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while(Math.abs(conveyorMotor.getCurrentPosition() - conveyorMotor.getTargetPosition()) > 10){
            AbstractOpMode.currentOpMode().telemetry.addData("", conveyorMotor.getCurrentPosition());
            AbstractOpMode.currentOpMode().telemetry.addData("", conveyorMotor.getTargetPosition());
            AbstractOpMode.currentOpMode().telemetry.update();

            conveyorMotor.setPower(-power);
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
            NormalizedRGBA colors = sensor.getNormalizedColors();
            double green = colors.green;
            double blue = colors.blue;

            if (green > 0.9) {
                if (blue > 0.9) {
                    detectedElement = true;
                } else {
                    detectedElement = true;
                }
            } else {
                detectedElement = false;
            }

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
            NormalizedRGBA colors = sensor.getNormalizedColors();
            double green = colors.green;
            double blue = colors.blue;

            if (green > 0.9) {
                if (blue > 0.9) {
                    detectedElement = true;
                } else {
                    detectedElement = true;
                }
            } else {
                detectedElement = false;
            }

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
        intakeDumb(0);
        house.setPosition(HOUSING_POSITION);
        Utils.sleep(250);
        linkage.setPosition(LINKAGE_HOUSED);
        stage = Stage.HOUSED;


    }

    public boolean isLinkageInPreScore(){
        return linkage.getPosition() != LINKAGE_SCORE;
    }



    public void raise(double position) {
        if(linkage.getPosition() != LINKAGE_HOUSED){
            preScore();
        }
        moveSlideNew(SLIDE_POWER, (int) position);
        linkage.setPosition(LINKAGE_SCORE);
        stage = stage.EXTENDED;
    }

    //temporary tele op scoring function w/o color sensor
    public synchronized void score(){
        house.setPosition(SCORING_POSITION);
    }

    public void retract(){
        moveSlide(-1, -100);
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
        AbstractOpMode.currentOpMode().telemetry.clear();
        if(winchEncoder.getCurrentPosition() - position > 0){
            Debug.log("extending");
            while (Math.abs(winchEncoder.getCurrentPosition() - position) > 100) {
                winchMotor.setPower(power);
            }

        }else{
            Debug.log("retracting");
            while (winchEncoder.getCurrentPosition() - position < -100) {
                winchMotor.setPower(power);
            }
        }

        winchMotor.setPower(0);

    }

    public void moveSlideNew(double power, int position){
        winchEncoder.setTargetPosition(position);
        winchEncoder.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        winchMotor.setPower(-power);
        while(Math.abs(winchEncoder.getCurrentPosition() - winchEncoder.getTargetPosition()) > 100){

        }
        winchMotor.setPower(0);
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
