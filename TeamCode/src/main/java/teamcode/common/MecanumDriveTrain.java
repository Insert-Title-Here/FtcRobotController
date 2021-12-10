package teamcode.common;

import android.sax.StartElementListener;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.robot.Robot;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.openftc.easyopencv.OpenCvWebcam;
import org.openftc.revextensions2.ExpansionHubMotor;

import java.util.Vector;

import teamcode.test.MasonTesting.CvDetectionPipeline;

public class MecanumDriveTrain {
    private static final double ANGULAR_TOLERANCE = 0.05;
    final double COUNTS_PER_INCH = 920.111004;

    /*
    This has most of the relevant information regarding a 4 wheel Mecanum DriveTrain,
    which is the most used DriveTrain in FTC
     */

    private ExpansionHubMotor fl, fr, bl, br;
    Localizer localizer;
    Vector2D previousVelocity;
    Vector2D previousError;
    double previousOmegaError;

    DistanceSensor distance;

    Logger logger;


    /**
     * PID Constants
     *
     */
    final double pVelocity = 0.000725; //0.000725
    final double dVelocity  = 0.047; //0.027

    public MecanumDriveTrain(HardwareMap hardwareMap){
        fl = (ExpansionHubMotor) hardwareMap.dcMotor.get("FrontLeftDrive");
        fr = (ExpansionHubMotor) hardwareMap.dcMotor.get("FrontRightDrive");
        bl = (ExpansionHubMotor) hardwareMap.dcMotor.get("BackLeftDrive");
        br = (ExpansionHubMotor) hardwareMap.dcMotor.get("BackRightDrive");
        correctMotors();

    }

    public MecanumDriveTrain(HardwareMap hardwareMap, Localizer localizer){
        fl = (ExpansionHubMotor) hardwareMap.dcMotor.get("FrontLeftDrive");
        fr = (ExpansionHubMotor) hardwareMap.dcMotor.get("FrontRightDrive");
        bl = (ExpansionHubMotor) hardwareMap.dcMotor.get("BackLeftDrive");
        br = (ExpansionHubMotor) hardwareMap.dcMotor.get("BackRightDrive");
        distance = hardwareMap.get(DistanceSensor.class, "Distance");
        this.localizer = localizer;
        previousVelocity = new Vector2D(0,0);
        previousOmega = 0;
        correctMotors();

    }

    CvDetectionPipeline pipeline;

    public MecanumDriveTrain(HardwareMap hardwareMap, Localizer localizer, CvDetectionPipeline pipeline){
        fl = (ExpansionHubMotor) hardwareMap.dcMotor.get("FrontLeftDrive");
        fr = (ExpansionHubMotor) hardwareMap.dcMotor.get("FrontRightDrive");
        bl = (ExpansionHubMotor) hardwareMap.dcMotor.get("BackLeftDrive");
        br = (ExpansionHubMotor) hardwareMap.dcMotor.get("BackRightDrive");
        distance = hardwareMap.get(DistanceSensor.class, "Distance");
        this.pipeline = pipeline;
        this.localizer = localizer;
        previousVelocity = new Vector2D(0,0);
        previousOmega = 0;
        correctMotors();

    }


    /**
     * DO NOT CALL THIS W/O ALL THE RIGHT CONSTRUCTORS
     * @param desiredVelocity
     * @param desiredRotate
     */
    public void seekCubes(double desiredVelocity, double desiredRotate){
        if(pipeline == null || localizer == null){
            return;
        }
        RobotPositionStateUpdater.RobotPositionState currentState = localizer.getCurrentState();
        Vector2D robotPosition;
        seekCubesRotate(desiredRotate);
        Vector2D desiredPosition;
        while(pipeline.yPointList().get(0) < 5) { //todo ask mason how to ensure I am tracking the same cube every time here
            //todo idea about above, write a method that traverses the stack by placing it in an arrayList, calculating the smallest deviation from the originally stored value and assuming that is the target,
            //this would dynamically adapt to the closest cube in the frame may cause some oscillation especially during the rotational phase.
            currentState = localizer.getCurrentState();
            robotPosition = new Vector2D(currentState.getPosition().getX(), currentState.getPosition().getY());

            Vector2D cubeToRobotDisplacement = new Vector2D(pipeline.xPointList().get(0), pipeline.yPointList().get(0));
            double cubeToRobotDisplacementMag = cubeToRobotDisplacement.magnitude();
            Vector2D cubeToRobotDisplacementOriented = new Vector2D(robotPosition.getDirection(), cubeToRobotDisplacementMag);
            desiredPosition = robotPosition.add(cubeToRobotDisplacementOriented);

            currentState = localizer.getCurrentState();
            Vector2D positionError = desiredPosition.subtract(currentState.getPosition());
            double errorAngle = positionError.getDirection();
            //angleOfTravel += 0; // (Math.PI / 4.0)mecanum need this because all the math is shifted by pi/4
            Vector2D idealVelocity = Vector2D.fromAngleMagnitude(errorAngle, desiredVelocity);

            Vector2D recordedVelocity = currentState.getVelocity();
            //recordedVelocity.rotate(-Math.PI / 4.0);

            double xError = (idealVelocity.getX() - recordedVelocity.getX());
            double yError = (idealVelocity.getY() - recordedVelocity.getY());
            Vector2D error = new Vector2D(xError, yError);
            //Vector2D crossTrackError = new Vector2D(xError, yError);
            Vector2D deltaError = error.subtract(previousError);
            error = error.multiply(pVelocity);
            deltaError = deltaError.multiply(dVelocity);
            error.add(deltaError);




            //found and fixed stupid math error
            Vector2D passedVector = previousVelocity.add(new Vector2D(error.getX(), error.getY()));
            if(Math.abs(fl.getPower()) == 1.0 || Math.abs(fr.getPower()) == 1.0 || Math.abs(bl.getPower()) == 1.0 || Math.abs(br.getPower()) == 1.0){
                passedVector = new Vector2D(previousVelocity.getX(), previousVelocity.getY());
                desiredVelocity = passedVector.magnitude();
            }
//            Vector2D maxVector = new Vector2D(Math.cos(direction), Math.sin(direction));
//            if(Math.abs(maxVector.getX()) < Math.abs(passedX)){
//                if(getSign(maxVector.getX()) == getSign(passedX)){
//                    passedX = maxVector.getX();
//                }else{
//                    passedX = -maxVector.getX();
//                }
//            }
//            if(Math.abs(maxVector.getY()) < Math.abs(passedY)){
//                if(getSign(maxVector.getY()) == getSign(passedY)){
//                    passedY = maxVector.getY();
//                }else{
//                    passedY = -maxVector.getY();
//                }
//            }

            //Vector2D passedVector = new Vector2D(passedX, passedY);
            previousVelocity = setVelocity(passedVector,0);

            // previousVelocity.multiply(sign);
            previousError = error;


            //AbstractOpMode.currentOpMode().telemetry.addData("", currentState.toString());
//
            //AbstractOpMode.currentOpMode().telemetry.addData("distance", Math.abs(newDesiredPosition.subtract(currentState.getPosition()).magnitude()));
            //AbstractOpMode.currentOpMode().telemetry.addData("sign", Math.abs(newDesiredPosition.subtract(currentState.getPosition()).magnitude()));

            AbstractOpMode.currentOpMode().telemetry.addData("", currentState);
            //AbstractOpMode.currentOpMode().telemetry.addData("error", (Math.abs(newDesiredPosition.subtract(currentState.getPosition()).magnitude())));

            AbstractOpMode.currentOpMode().telemetry.update();
        }
    }

    private final double pOmega = 0;
    private void seekCubesRotate(double desiredOmega) {
        double xPartitionDeviation = pipeline.xPointList().get(0);
        previousOmega = 0;
        while(xPartitionDeviation > 5){
            xPartitionDeviation = pipeline.xPointList().get(0);
            double recordedOmega = localizer.getCurrentState().getAngularVelocity();
            double omegaError = desiredOmega - recordedOmega;
            omegaError *= pOmega;
            double passedOmega = omegaError + previousOmega;
            if(fl.getPower() == 1.0 || fr.getPower() == 1.0 || bl.getPower() == 1.0 || br.getPower() == 1.0){
                passedOmega = 1.0;
                desiredOmega = recordedOmega;
            }else if(fl.getPower() == -1.0 || fr.getPower() == -1.0 || bl.getPower() == -1.0 || br.getPower() == -1.0){
                passedOmega = -1.0;
                desiredOmega = recordedOmega;
            }
            setVelocity(new Vector2D(0,0), passedOmega);
            previousOmega = passedOmega;
        }

    }



    public void strafeDistanceSensor(double desiredVelocity){
        if(localizer == null){
            return;
        }
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
     * moving from position to position, assuming some kind of translational motion
     * NEW no rotation involved since we can handle that as a single dimensional operation seperately rather than integrating it into the 3 dimensional operation it was
     * @param desiredPosition the end point of the robot
     * @param desiredVelocity the end velocity of the robot in inches per second
     *
     */

    public void moveToPosition(Vector2D desiredPosition, double desiredVelocity){

        if(localizer == null){
            return;
        }
        RobotPositionStateUpdater.RobotPositionState currentState = localizer.getCurrentState();
        Vector2D desiredPositionPointer = new Vector2D(desiredPosition.getX() - currentState.getPosition().getX() , desiredPosition.getY() - currentState.getPosition().getY());
        Vector2D newDesiredPosition = desiredPosition.add(new Vector2D(5.0 * Math.cos(desiredPositionPointer.getDirection()), 5.0 * Math.sin(desiredPositionPointer.getDirection())));

        previousError = new Vector2D(0,0);
        Vector2D steadyStateError = new Vector2D(0,0);
        previousOmegaError = 0;


        while((Math.abs(newDesiredPosition.subtract(currentState.getPosition()).magnitude()) > 5.0) && AbstractOpMode.currentOpMode().opModeIsActive()){

            currentState = localizer.getCurrentState();
            Vector2D positionError = desiredPosition.subtract(currentState.getPosition());
            double errorAngle = positionError.getDirection();
            //angleOfTravel += 0; // (Math.PI / 4.0)mecanum need this because all the math is shifted by pi/4
            Vector2D idealVelocity = Vector2D.fromAngleMagnitude(errorAngle, desiredVelocity);

            Vector2D recordedVelocity = currentState.getVelocity();
            //recordedVelocity.rotate(-Math.PI / 4.0);

            double xError = (idealVelocity.getX() - recordedVelocity.getX());
            double yError = (idealVelocity.getY() - recordedVelocity.getY());
            Vector2D error = new Vector2D(xError, yError);
            //Vector2D crossTrackError = new Vector2D(xError, yError);
            steadyStateError.add(error);
            Vector2D deltaError = error.subtract(previousError);
            error = error.multiply(pVelocity);
            deltaError = deltaError.multiply(dVelocity);
            error.add(deltaError);




            //found and fixed stupid math error
            Vector2D passedVector = previousVelocity.add(new Vector2D(error.getX(), error.getY()));
            if(Math.abs(fl.getPower()) == 1.0 || Math.abs(fr.getPower()) == 1.0 || Math.abs(bl.getPower()) == 1.0 || Math.abs(br.getPower()) == 1.0){
                passedVector = new Vector2D(previousVelocity.getX(), previousVelocity.getY());
                desiredVelocity = passedVector.magnitude();
            }
//            Vector2D maxVector = new Vector2D(Math.cos(direction), Math.sin(direction));
//            if(Math.abs(maxVector.getX()) < Math.abs(passedX)){
//                if(getSign(maxVector.getX()) == getSign(passedX)){
//                    passedX = maxVector.getX();
//                }else{
//                    passedX = -maxVector.getX();
//                }
//            }
//            if(Math.abs(maxVector.getY()) < Math.abs(passedY)){
//                if(getSign(maxVector.getY()) == getSign(passedY)){
//                    passedY = maxVector.getY();
//                }else{
//                    passedY = -maxVector.getY();
//                }
//            }

            //Vector2D passedVector = new Vector2D(passedX, passedY);
            previousVelocity = setVelocity(passedVector,0);

           // previousVelocity.multiply(sign);
            previousError = error;


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
        return"fl: " + fl.getPower() + "\n" +
                "fr: " + fr.getPower() + "\n" +
                "bl: " + bl.getPower() + "\n" +
                "br: " + br.getPower();
    }

    public Vector2D setVelocity(Vector2D velocity, double turnValue){
        turnValue = -turnValue;
        double direction = velocity.getDirection();



        double power = velocity.magnitude();

        double angle = direction + (Math.PI / 4.0);
        double sin = Math.sin(angle);
        double cos = Math.cos(angle);

        setMotorVelocity((power * sin - turnValue),(power * cos - turnValue),
                (power * cos + turnValue), (power * sin + turnValue));
        return new Vector2D(velocity.getX(), velocity.getY());
    }
        //Vt = rw
    private final double WHEEL_RADIUS_IN = 1.88976; // radius of the 96mm gobilda wheels in IN
    public void setMotorVelocity(double flVelocity, double frVelocity, double blVelocity, double brVelocity){
        fl.setVelocity(flVelocity * WHEEL_RADIUS_IN, AngleUnit.RADIANS);
        fr.setVelocity(frVelocity * WHEEL_RADIUS_IN, AngleUnit.RADIANS);
        bl.setVelocity(blVelocity * WHEEL_RADIUS_IN, AngleUnit.RADIANS);
        br.setVelocity(brVelocity * WHEEL_RADIUS_IN, AngleUnit.RADIANS);
    }


    double previousOmega;
    double pRotation;
    public void moveToRotation(double desiredRotation, double omega){
        if(localizer == null){
            return;
        }
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

        double angle = direction + (3 * Math.PI / 4.0);
        double sin = Math.sin(angle);
        double cos = Math.cos(angle);

        setPower((power * sin - turnValue),(power * cos + turnValue),
                (power * cos - turnValue), (power * sin + turnValue));
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
        fl.setPower(-flPow);
        fr.setPower(frPow);
        bl.setPower(-blPow);
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
