package teamcode.Competition;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.List;

import teamcode.common.Localizer;
import teamcode.common.RobotPositionStateUpdater;
import teamcode.common.Utils;

public class ArmSystem {
    private static final double TOP_POSITION = 15.5;
    private static final double LOW_POSITION = 4.5;
    private static final double MEDIUM_POSITION = 8.5;
    private static final double BOTTOM_POSITION = 0;

    //House Servo values
    private static final double INTAKE_POSITION = 0;
    private static final double HOUSING_POSITION = 0; //TODO calibrate both these values
    private static final double SCORING_POSITION = 0;

    private static final double LINKAGE_DOWN = 0;
    private static final double LINKAGE_SCORE = 0.5;

    private static final float GREEN_THRESHOLD = 255;
    private static final float RED_THRESHOLD = 255;
    private static final float BLUE_THRESHOLD = 255;
    private static final int YELLOW_THRESHOLD = 02552550;
    private static final int WHITE_THRESHOLD = 0255255255;

    private static final double CONVEYOR_POWER = 0.8;
    private static final double SLIDE_POWER = 0.6;

    private Localizer localizer;
    private DcMotor leftIntake, rightIntake, winchMotor, conveyorMotor;
    private Servo house, linkage;
    RobotPositionStateUpdater.RobotPositionState currentState;
    private Stage stage;


    public ArmSystem(HardwareMap hardwareMap, Localizer localizer, boolean isTeleOp){
        leftIntake = hardwareMap.dcMotor.get("leftIntake");
        rightIntake = hardwareMap.dcMotor.get("rightIntake");
        winchMotor = hardwareMap.dcMotor.get("winch");
        conveyorMotor = hardwareMap.dcMotor.get("conveyor");
        house = hardwareMap.servo.get("house");
        linkage = hardwareMap.servo.get("Parallelogram");
        this.localizer = localizer;
        currentState = localizer.getCurrentState();
        if(isTeleOp){
            house.setPosition(INTAKE_POSITION);
        }else{
            house.setPosition(HOUSING_POSITION);
        }
        linkage.setPosition(LINKAGE_DOWN);
        stage = Stage.IDLE;
    }

    public void intakeDumb(double power){
        leftIntake.setPower(power);
        rightIntake.setPower(-power);
    }

    public void intake(double intakePower){
        house.setPosition(INTAKE_POSITION);
        intakeDumb(intakePower);
        stage = Stage.INTAKING;
        NormalizedRGBA houseRGBA = localizer.getCurrentState().getHouseRGBA();
        while(houseRGBA.green < GREEN_THRESHOLD && houseRGBA.red < RED_THRESHOLD){ //TODO replace with actual conditional that is true for both balls and cubes
            houseRGBA = localizer.getCurrentState().getHouseRGBA();
        }
        if(houseRGBA.blue >= BLUE_THRESHOLD){
            //balls



            stage = Stage.BALL_HOUSED;

        }else{
            //cubes
            stage = Stage.CUBE_HOUSED;
        }

        linkage.setPosition(LINKAGE_SCORE);
        Utils.sleep(100);
        house.setPosition(HOUSING_POSITION);
    }

    private enum Stage{
        INTAKING, IDLE, BALL_HOUSED, CUBE_HOUSED
    }

    //tele op scoring function, assumes the freight is encapsulated in the house already and that the
    //linkage is raised (not scoring). This method also assumes the Conveyor exists as well so if we
    //get rid of the conveyor we need to change this
    public void score(){
        if(stage == Stage.CUBE_HOUSED) {
            NormalizedRGBA conveyorRGBA = localizer.getCurrentState().getConveyorRGBA();
            conveyorMotor.setPower(CONVEYOR_POWER);
            while(conveyorRGBA.toColor() < YELLOW_THRESHOLD){
                conveyorRGBA = localizer.getCurrentState().getConveyorRGBA();
            }
            Utils.sleep(1000);
            conveyorMotor.setPower(0);
        }else if(stage == Stage.BALL_HOUSED){
            moveSlide(SLIDE_POWER, TOP_POSITION);

            linkage.setPosition(LINKAGE_SCORE);
            Utils.sleep(200);
            house.setPosition(SCORING_POSITION);

            Utils.sleep(500);
            moveSlide(-SLIDE_POWER, BOTTOM_POSITION);

        }
        house.setPosition(INTAKE_POSITION);
        linkage.setPosition(LINKAGE_DOWN);
        stage = Stage.IDLE;

    }

    public void moveSlide(double power, double position){
        while (Math.abs(currentState.getLinearSlidePosition() - position) > 0.1) {
            currentState = localizer.getCurrentState();
            winchMotor.setPower(power);
        }
        winchMotor.setPower(0);

    }

    //needs to be rewritten if the conveyor is implemented
    public void score(BarcodeReaderPipeline.BarcodePosition position){
        if(position == BarcodeReaderPipeline.BarcodePosition.LEFT){
            moveSlide(SLIDE_POWER, LOW_POSITION);

        }else if(position == BarcodeReaderPipeline.BarcodePosition.CENTER){
            moveSlide(SLIDE_POWER, MEDIUM_POSITION);

        }else if(position == BarcodeReaderPipeline.BarcodePosition.RIGHT){
            moveSlide(SLIDE_POWER, TOP_POSITION);
        }
        linkage.setPosition(LINKAGE_SCORE);
        Utils.sleep(200);
        house.setPosition(SCORING_POSITION);

        Utils.sleep(500);

        moveSlide(-SLIDE_POWER, BOTTOM_POSITION);

    }

    public Stage getStage(){
        return stage;
    }

}
