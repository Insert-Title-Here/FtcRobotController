package teamcode.common;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.openftc.revextensions2.ExpansionHubMotor;

import java.util.concurrent.AbstractExecutorService;

public class WestCoastDriveTrain {

    private ExpansionHubMotor fl, fr, bl, br;
    Localizer localizer;

    private final double P_LINEAR = 0.0007;
    private final double I_LINEAR = 0;
    private final double D_LINEAR = 0;


    private final double P_ROTATIONAL = 0.02;
    private final double I_ROTATIONAL = 0;
    private final double D_ROTATIONAL = 0;

    private double previousVelocity;
    private double previousOmega;
    private double previousError;
    private double previousOmegaError;

    private final double WHEEL_RADIUS = 0; //TODO measure


    public WestCoastDriveTrain(Localizer localizer, HardwareMap hardwareMap){

    }

    public WestCoastDriveTrain(HardwareMap hardwareMap){
        fl = hardwareMap.get(ExpansionHubMotor.class,"FrontLeftDrive");
        fr = hardwareMap.get(ExpansionHubMotor.class, "FrontRightDrive");
        bl = hardwareMap.get(ExpansionHubMotor.class, "BackLeftDrive");
        br = hardwareMap.get(ExpansionHubMotor.class, "BackRightDrive");
        correctMotors();

    }

    public WestCoastDriveTrain(HardwareMap hardwareMap, Localizer localizer){
        fl = hardwareMap.get(ExpansionHubMotor.class,"FrontLeftDrive");
        fr = hardwareMap.get(ExpansionHubMotor.class, "FrontRightDrive");
        bl = hardwareMap.get(ExpansionHubMotor.class, "BackLeftDrive");
        br = hardwareMap.get(ExpansionHubMotor.class, "BackRightDrive");
        this.localizer = localizer;
        previousVelocity = 0;
        previousOmega = 0;
        correctMotors();

    }

    //TODO implement correctly
    private void correctMotors() {


    }

    public void moveToPosition(Vector2D desiredPosition, double desiredVelocity, double desiredOmega){


        RobotPositionStateUpdater.RobotPositionState currentState = localizer.getCurrentState();

        Vector2D desiredPositionPointer = new Vector2D(desiredPosition.getX() - currentState.getPosition().getX() , desiredPosition.getY() - currentState.getPosition().getY());
        Vector2D newDesiredPosition = desiredPosition.add(new Vector2D(5.0 * Math.cos(desiredPositionPointer.getDirection()), 5.0 * Math.sin(desiredPositionPointer.getDirection())));

        previousError =  0;
        previousVelocity = 0;
        double steadyStateError = 0;
        //moveToRotation( newDesiredPosition.getDirection(), desiredOmega);

        long previousTimeMillis = System.currentTimeMillis();

        while((Math.abs(newDesiredPosition.subtract(currentState.getPosition()).magnitude()) > 5.0 && AbstractOpMode.currentOpMode().opModeIsActive())){
            long currentCycleTimeMillis = System.currentTimeMillis() - previousTimeMillis;
            double currentCycleTimeSeconds = currentCycleTimeMillis / 1000.0;
            previousTimeMillis = System.currentTimeMillis();
            currentState = localizer.getCurrentState();
            Vector2D positionError = desiredPosition.subtract(currentState.getPosition());
            double errorAngle = positionError.getDirection();
            Vector2D idealVelocity = Vector2D.fromAngleMagnitude(errorAngle, desiredVelocity);

            double recordedVelocity = currentState.getVelocity().magnitude();


            double error = idealVelocity.magnitude() - recordedVelocity;
            steadyStateError += (error * currentCycleTimeSeconds);
            double deltaError = (error - previousError) / currentCycleTimeSeconds;

            double output = error * P_LINEAR;
// + deltaError * D_LINEAR + steadyStateError * I_LINEAR


            double passedValue = output + previousVelocity;

            if(passedValue > 1.0){
                passedValue = 1.0;
            }else if(passedValue < -1.0){
                passedValue = -1.0;
            }


            previousVelocity = straightMovement(passedValue);
            previousError = error;


            AbstractOpMode.currentOpMode().telemetry.addData("passedVelocity", passedValue);
            AbstractOpMode.currentOpMode().telemetry.addData("distance", Math.abs(newDesiredPosition.subtract(currentState.getPosition()).magnitude()));
            //AbstractOpMode.currentOpMode().telemetry.addData("sign", Math.abs(newDesiredPosition.subtract(currentState.getPosition()).magnitude()));
            AbstractOpMode.currentOpMode().telemetry.addData("", currentState);
            AbstractOpMode.currentOpMode().telemetry.addData("error", (Math.abs(newDesiredPosition.subtract(currentState.getPosition()).magnitude())));
            AbstractOpMode.currentOpMode().telemetry.update();
        }
        brake();
    }

    private void brake() {
        fl.setPower(0);
        fr.setPower(0);
        bl.setPower(0);
        br.setPower(0);
        previousVelocity = 0;
    }


    public double straightMovement(double power){

        setPower(-power, power, -power, power);
        return power;
    }


    private int getSign(double num){
        if(num < 0){
            return -1;
        }else {
            return 1;
        }
    }


    public void setPower(double linear, double rotation){
        rotation = -rotation;


        fl.setPower(linear + rotation);
        fr.setPower(-linear + rotation);
        bl.setPower(linear + rotation);
        br.setPower(-linear + rotation);

        AbstractOpMode.currentOpMode().telemetry.addData("fl", fl.getPower());
        AbstractOpMode.currentOpMode().telemetry.addData("fl", fr.getPower());
        AbstractOpMode.currentOpMode().telemetry.addData("fl", bl.getPower());
        AbstractOpMode.currentOpMode().telemetry.addData("fl", br.getPower());
        AbstractOpMode.currentOpMode().telemetry.update();
    }

//TODO reread this and the linear one
    public void moveToRotation(double desiredRotation, double desiredOmega) {
        RobotPositionStateUpdater.RobotPositionState currentState = localizer.getCurrentState();
        double currentRotation = currentState.getRotation();
        double newDesiredRotation = desiredRotation;
        if(desiredOmega > 0){
            newDesiredRotation += 0.05;
        }else{
            newDesiredRotation -= 0.05;
        }
        previousOmegaError = 0;
        double steadyStateError = 0;
        long previousTimeMillis = System.currentTimeMillis();

        while(Math.abs(newDesiredRotation - currentRotation) > 0.05 && AbstractOpMode.currentOpMode().opModeIsActive()){
            currentState = localizer.getCurrentState();
            double recordedOmega = currentState.getAngularVelocity();
            long currentCycleTimeMillis = System.currentTimeMillis() - previousTimeMillis;
            double currentCycleTimeSeconds = currentCycleTimeMillis / 1000.0;
            previousTimeMillis = System.currentTimeMillis();

            double omegaError = desiredOmega - recordedOmega;
            double deltaOmegaError = (omegaError - previousOmegaError) / currentCycleTimeSeconds;
            steadyStateError += omegaError * currentCycleTimeSeconds;

            double passedOmega = previousOmega + omegaError * P_ROTATIONAL + steadyStateError * I_ROTATIONAL + deltaOmegaError * D_ROTATIONAL;

            if(passedOmega > 1.0){
                passedOmega = 1.0;
            }else if(passedOmega < -1.0){
                passedOmega = -1.0;
            }

            previousOmega = rotate(passedOmega);
        }
    }

    public double rotate(double omega) {
        setPower(-omega, -omega, -omega, -omega);
        return -omega;
    }

    public void setPower(double flPower, double frPower, double blPower, double brPower){
        fl.setPower(flPower);
        fr.setPower(frPower);
        bl.setPower(blPower);
        br.setPower(brPower);

    }

    public void setVelocity(double flVelocity, double frVelocity, double blVelocity, double brVelocity){
        fl.setVelocity(flVelocity);
        fr.setVelocity(frVelocity);
        bl.setVelocity(blVelocity);
        br.setVelocity(brVelocity);
    }




}
