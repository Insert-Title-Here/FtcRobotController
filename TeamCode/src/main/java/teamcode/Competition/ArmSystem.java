package teamcode.Competition;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;

import teamcode.common.Localizer;
import teamcode.common.RobotPositionStateUpdater;
import teamcode.common.Utils;

public class ArmSystem {
    private static final double TOP_POSITION = 15.5;
    private static final double BOTTOM_POSITION = 0;

    private static final double INTAKE_POSITION = 0;
    private static final double HOUSING_POSITION = 0; //TODO calibrate both these values
    private static final double SCORING_POSITION = 0;

    private static final float GREEN_THRESHOLD = 255;
    private static final float RED_THRESHOLD = 255;
    private static final float BLUE_THRESHOLD = 255;
    private static final int YELLOW_THRESHOLD = 02552550;
    private static final int WHITE_THRESHOLD = 0255255255;

    private static final double CONVEYOR_POWER = 0.8;

    private Localizer localizer;
    private DcMotor leftIntake, rightIntake, winchMotor, conveyorMotor;
    private Servo house;
    private NormalizedColorSensor houseSensor, conveyorSensor;
    RobotPositionStateUpdater.RobotPositionState currentState;
    private Stage stage;

    public ArmSystem(HardwareMap hardwareMap, Localizer localizer, boolean isTeleOp){
        leftIntake = hardwareMap.dcMotor.get("leftIntake");
        rightIntake = hardwareMap.dcMotor.get("rightIntake");
        winchMotor = hardwareMap.dcMotor.get("winch");
        conveyorMotor = hardwareMap.dcMotor.get("conveyor");
        house = hardwareMap.servo.get("house");
        houseSensor = hardwareMap.get(NormalizedColorSensor.class, "houseSensor");
        conveyorSensor = hardwareMap.get(NormalizedColorSensor.class, "conveyorSensor");
        this.localizer = localizer;
        currentState = localizer.getCurrentState();
        if(isTeleOp){
            house.setPosition(INTAKE_POSITION);
        }else{
            house.setPosition(HOUSING_POSITION);
        }
        stage = Stage.IDLE;
    }

    public void intakeDumb(double power){
        leftIntake.setPower(power);
        rightIntake.setPower(-power);
    }

    public void intake(double intakePower, double slidePower){
        intakeDumb(intakePower);
        NormalizedRGBA houseRGBA = localizer.getCurrentState().getHouseRGBA();

        while(houseRGBA.green < GREEN_THRESHOLD && houseRGBA.red < RED_THRESHOLD){ //TODO replace with actual conditional that is true for both balls and cubes
            houseRGBA = localizer.getCurrentState().getHouseRGBA();
            stage = Stage.INTAKING;
        }
        if(houseRGBA.blue >= BLUE_THRESHOLD){
            //balls
            while (Math.abs(currentState.getLinearSlidePosition() - TOP_POSITION) > 0.1) {
                currentState = localizer.getCurrentState();
                winchMotor.setPower(slidePower);
            }
            winchMotor.setPower(0);
            stage = Stage.BALL_HOUSED;

        }else{
            //cubes

            stage = Stage.CUBE_HOUSED;
        }
    }

    private enum Stage{
        INTAKING, IDLE, BALL_HOUSED, CUBE_HOUSED
    }


    public void score(double slidePower){
        if(stage == Stage.CUBE_HOUSED) {
            NormalizedRGBA conveyorRGBA = localizer.getCurrentState().getConveyorRGBA();
            conveyorMotor.setPower(CONVEYOR_POWER);
            while(conveyorRGBA.toColor() < YELLOW_THRESHOLD){
                conveyorRGBA = localizer.getCurrentState().getConveyorRGBA();
            }
            Utils.sleep(1000);
            conveyorMotor.setPower(0);
        }else if(stage == Stage.BALL_HOUSED){
            house.setPosition(SCORING_POSITION);
            Utils.sleep(500);
            while (Math.abs(currentState.getLinearSlidePosition() - BOTTOM_POSITION) > 0.1) {
                currentState = localizer.getCurrentState();
                winchMotor.setPower(-slidePower);
            }
            winchMotor.setPower(0);
        }

    }

    public Stage getStage(){
        return stage;
    }

}
