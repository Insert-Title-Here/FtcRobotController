package teamcode.common;

import android.sax.StartElementListener;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.robot.Robot;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.Vector;

public class MecanumDriveTrain {
    private static final double ANGULAR_TOLERANCE = 0.05;
    final double COUNTS_PER_INCH = 920.111004;

    /*
    This has most of the relevant information regarding a 4 wheel Mecanum DriveTrain,
    which is the most used DriveTrain in FTC
     */

    private DcMotor fl, fr, bl, br;
    Localizer localizer;
    Vector2D previousVelocity;
    Vector2D previousError;
    double previousOmegaError;

    DistanceSensor distance;


    /**
     * PID Constants
     *
     */
    final double pVelocity = 0.000725; //0.000725
    final double dVelocity  = 0.047; //0.027
    final double pOmega = 0.1;
    final double dOmega = 0;

    public MecanumDriveTrain(HardwareMap hardwareMap){
        fl = hardwareMap.dcMotor.get("FrontLeftDrive");
        fr = hardwareMap.dcMotor.get("FrontRightDrive");
        bl = hardwareMap.dcMotor.get("BackLeftDrive");
        br = hardwareMap.dcMotor.get("BackRightDrive");
        correctMotors();

    }

    public MecanumDriveTrain(HardwareMap hardwareMap, Localizer localizer){
        fl = hardwareMap.dcMotor.get("FrontLeftDrive");
        fr = hardwareMap.dcMotor.get("FrontRightDrive");
        bl = hardwareMap.dcMotor.get("BackLeftDrive");
        br = hardwareMap.dcMotor.get("BackRightDrive");
        distance = hardwareMap.get(DistanceSensor.class, "Distance");
        this.localizer = localizer;
        previousVelocity = new Vector2D(0,0);
        previousOmega = 0;
        correctMotors();

    }

    public void strafeDistanceSensor(double desiredVelocity){
        RobotPositionStateUpdater.RobotPositionState currentState = localizer.getCurrentState();

        double previousError = 0;
        double previousVelocity = 0;
        Vector2D steadyStateError = new Vector2D(0,0);
        previousOmegaError = 0;

        //todo calibrate the tolerance of it.
        while(distance.getDistance(DistanceUnit.INCH) > 0.5 && AbstractOpMode.currentOpMode().opModeIsActive()){

            currentState = localizer.getCurrentState();

            //angleOfTravel += 0; // (Math.PI / 4.0)mecanum need this because all the math is shifted by pi/4

            Vector2D recordedVelocity = currentState.getVelocity();
            double recordedVelocityMag = recordedVelocity.magnitude();
            //recordedVelocity.rotate(-Math.PI / 4.0);

            double error = desiredVelocity - recordedVelocityMag;
            //Vector2D crossTrackError = new Vector2D(xError, yError);
            //steadyStateError.add(error);
            double deltaError = error - previousError;
            error = error * pVelocity;
            deltaError = deltaError * dVelocity;
            error += deltaError;


//            double sign = 1.0;
//            if(error.getX() < 0 || error.getY() < 0){
//                sign = -1.0;
//            }
            //error.add(previousVelocity);

            //found and fixed stupid math error
            double passedVal = previousVelocity + error;


            if(passedVal > 1){
                passedVal = 1;
                desiredVelocity = currentState.getVelocity().magnitude();
            }
            if(passedVal < -1){
                passedVal = -1;
                desiredVelocity = currentState.getVelocity().magnitude();
            }

            //Vector2D passedVector = new Vector2D(passedX, passedY);
            previousVelocity = setStrafe(passedVal);

            // previousVelocity.multiply(sign);
            previousError = error;

        }
        brake();



    }


    private void correctMotors() {
        fl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }






    /**
     * moving from position to position
     * @param desiredPosition the end point of the robot
     * @param desiredVelocity the end velocity of the robot in inches per second
     * @param desiredAngle the angle the robot should be at at the end in radians.
     * @param desiredOmega the rate of angle change in rads/sec
     */

    public void moveToPosition(Vector2D desiredPosition, double desiredVelocity, double desiredAngle, double desiredOmega){


        RobotPositionStateUpdater.RobotPositionState currentState = localizer.getCurrentState();

        Vector2D desiredPositionPointer = new Vector2D(desiredPosition.getX() - currentState.getPosition().getX() , desiredPosition.getY() - currentState.getPosition().getY());
        Vector2D newDesiredPosition = desiredPosition.add(new Vector2D(5.0 * Math.cos(desiredPositionPointer.getDirection()), 5.0 * Math.sin(desiredPositionPointer.getDirection())));

        previousError = new Vector2D(0,0);
        Vector2D steadyStateError = new Vector2D(0,0);
        previousOmegaError = 0;
        double newDesiredAngle = desiredAngle;


        while((Math.abs(newDesiredPosition.subtract(currentState.getPosition()).magnitude()) > 5.0 || Math.abs(currentState.getRotation() - newDesiredAngle) > 0.05) && AbstractOpMode.currentOpMode().opModeIsActive()){

            currentState = localizer.getCurrentState();
            Vector2D positionError = desiredPosition.subtract(currentState.getPosition());
            double errorAngle = positionError.getDirection();
            //angleOfTravel += 0; // (Math.PI / 4.0)mecanum need this because all the math is shifted by pi/4
            Vector2D idealVelocity = Vector2D.fromAngleMagnitude(errorAngle, desiredVelocity);

            Vector2D recordedVelocity = currentState.getVelocity();
            //recordedVelocity.rotate(-Math.PI / 4.0);

            double currentRotation = currentState.getRotation();
            double rotationError =  (newDesiredAngle - currentRotation);


            double recordedOmega = currentState.getAngularVelocity();

            double xError = (idealVelocity.getX() - recordedVelocity.getX());
            double yError = (idealVelocity.getY() - recordedVelocity.getY());
            double omegaError =  (desiredOmega - recordedOmega);
            Vector2D error = new Vector2D(xError, yError);
            //Vector2D crossTrackError = new Vector2D(xError, yError);
            steadyStateError.add(error);
            double deltaErrorOmega = omegaError - previousOmegaError;
            Vector2D deltaError = error.subtract(previousError);
            error = error.multiply(pVelocity);
            deltaError = deltaError.multiply(dVelocity);
            error.add(deltaError);

            omegaError *= pOmega;
            deltaErrorOmega *= dOmega;
            omegaError += deltaErrorOmega;



//            double sign = 1.0;
//            if(error.getX() < 0 || error.getY() < 0){
//                sign = -1.0;
//            }
            //error.add(previousVelocity);
            double direction = error.getDirection();

            //found and fixed stupid math error
            Vector2D passedVector = previousVelocity.add(new Vector2D(error.getX(), error.getY()));
            double passedX = passedVector.getX();
            double passedY = passedVector.getY();

            Vector2D maxVector = new Vector2D(Math.cos(direction), Math.sin(direction));
            if(Math.abs(maxVector.getX()) < Math.abs(passedX)){
                if(getSign(maxVector.getX()) == getSign(passedX)){
                    passedX = maxVector.getX();
                }else{
                    passedX = -maxVector.getX();
                }
            }
            if(Math.abs(maxVector.getY()) < Math.abs(passedY)){
                if(getSign(maxVector.getY()) == getSign(passedY)){
                    passedY = maxVector.getY();
                }else{
                    passedY = -maxVector.getY();
                }
            }
            passedVector = new Vector2D(passedX, passedY);

            omegaError += previousOmega;
            //Vector2D passedVector = new Vector2D(passedX, passedY);
            previousVelocity = setPowerPurePursuit(passedVector, omegaError);

           // previousVelocity.multiply(sign);
            previousError = error;
            previousOmegaError = omegaError;

            //AbstractOpMode.currentOpMode().telemetry.addData("", currentState.toString());
//
            //AbstractOpMode.currentOpMode().telemetry.addData("distance", Math.abs(newDesiredPosition.subtract(currentState.getPosition()).magnitude()));
            //AbstractOpMode.currentOpMode().telemetry.addData("sign", Math.abs(newDesiredPosition.subtract(currentState.getPosition()).magnitude()));

            AbstractOpMode.currentOpMode().telemetry.addData("", currentState);
            //AbstractOpMode.currentOpMode().telemetry.addData("error", (Math.abs(newDesiredPosition.subtract(currentState.getPosition()).magnitude())));

            AbstractOpMode.currentOpMode().telemetry.update();
        }
        brake();


    }

    public String getMotorPower(){
        return "fl: " + fl.getPower() + "\n" +
                "fr: " + fr.getPower() + "\n" +
                "bl: " + bl.getPower() + "\n" +
                "br: " + br.getPower();
    }


    double previousOmega;
    double pRotation;
    public void moveToRotation(double desiredRotation, double omega){
        RobotPositionStateUpdater.RobotPositionState state = localizer.getCurrentState();
        previousOmega = 0;
        while(Math.abs(desiredRotation - state.getRotation()) > 0.05){
            state = localizer.getCurrentState();
            double recordedOmega = state.getAngularVelocity();
            double omegaError = omega - recordedOmega;
            omegaError *= pRotation;
            omega += omegaError;
            setPowerPurePursuit(new Vector2D(0,0), omega);
//            AbstractOpMode.currentOpMode().telemetry.addData("", state.toString());
//            AbstractOpMode.currentOpMode().telemetry.update();
        }
        brake();
    }

    private void brake() {
        fl.setPower(0);
        fr.setPower(0);
        bl.setPower(0);
        br.setPower(0);
        previousVelocity = new Vector2D(0,0);
    }


    public DcMotor[] getMotors(){
        return new DcMotor[]{fl,fr,bl,br};
    }


    /*
    gets the robot driving in a specified direction
     */
    public void setPower(Vector2D velocity, double turnValue){
        turnValue = -turnValue;
        double direction = velocity.getDirection();



        double power = velocity.magnitude();

        double angle = direction + (Math.PI / 4.0);
        double sin = Math.sin(angle);
        double cos = Math.cos(angle);

        setPower((power * sin - turnValue),(power * cos - turnValue),
                (power * cos + turnValue), (power * sin + turnValue));
    }

    /*
    this exists because of the differences between the FTC controller and raw vectors
     */
    public Vector2D setPowerPurePursuit(Vector2D velocity, double turnValue){

        turnValue = turnValue;
        double direction = velocity.getDirection();
        double power = velocity.magnitude();

        double angle = direction +  Math.PI / 4;

        double sin = Math.sin(angle);
        double cos = Math.cos(angle);
        // right movement
        // fl +, fr -, bl -, br +
        // positive clockwise
        // fl +, fr -, bl +, br -
        setPowerCorrect(power * sin + turnValue, -power * cos - turnValue,
                -power * cos + turnValue, power * sin - turnValue);
        return velocity;
    }

    public void setPowerCorrect(double flPow, double frPow, double blPow, double brPow) {
        fl.setPower(flPow);
        fr.setPower(frPow);
        bl.setPower(-blPow);
        br.setPower(brPow);
    }

    public void setPower(double flPow, double frPow, double blPow, double brPow) {
        fl.setPower(flPow);
        fr.setPower(frPow);
        bl.setPower(blPow);
        br.setPower(brPow);
    }

    public double setStrafe(double val){
        fl.setPower(val);
        fr.setPower(-val);
        bl.setPower(-val);
        br.setPower(val);
        return val;
    }

    private boolean isNear(double globalRads, double angle, boolean isBig) {
        if (isBig) {
            return Math.abs(globalRads - angle) < (2 * ANGULAR_TOLERANCE);
        }else {
            return Math.abs(globalRads - angle) < (ANGULAR_TOLERANCE);
        }
    }

    public void zero() {
        setPower(0,0,0,0);
    }

    private int getSign(double num){
        if(num < 0){
            return -1;
        }else {
            return 1;
        }
    }









}
