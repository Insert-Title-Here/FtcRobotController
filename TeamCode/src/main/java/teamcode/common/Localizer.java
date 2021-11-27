package teamcode.common;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Transform2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ReadWriteFile;
import com.spartronics4915.lib.T265Camera;

import org.apache.commons.math3.analysis.function.Abs;
import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;
import org.apache.commons.math3.linear.Array2DRowRealMatrix;
import org.apache.commons.math3.linear.DecompositionSolver;
import org.apache.commons.math3.linear.LUDecomposition;
import org.apache.commons.math3.linear.MatrixUtils;
import org.apache.commons.math3.linear.RealMatrix;
import org.checkerframework.checker.units.qual.A;
import org.checkerframework.checker.units.qual.Angle;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.openftc.revextensions2.ExpansionHubEx;
import org.openftc.revextensions2.ExpansionHubMotor;
import org.openftc.revextensions2.RevBulkData;

import java.io.File;
import java.io.FileNotFoundException;
import java.io.PrintStream;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Vector;
import java.util.concurrent.atomic.AtomicBoolean;

import teamcode.common.PositionStuff.Point;
import teamcode.common.PositionStuff.Pose;
import teamcode.common.PurePursuit.MathFunctions;

import static java.lang.Math.*;

public class Localizer extends Thread {
    //TODO before reading this file please note the static import of the math class,
    // odds are if you see a math function it is from that and not a constatnt/method I created
    //https://docs.google.com/document/d/1JQuU2M--rVFEa9ApKkOcc0CalsBxKjH-l0PQLRW7F3c/edit?usp=sharing proof behind the math

    //odometry wheel constants, MUST BE CALIBRATED FOR EACH ROBOT
    private static final double TICKS_PER_REV = 8192;
    private static final double WHEEL_DIAMETER = 1.378; //1.181 for 60 mm, 1.417 for 72mm
    private static final double GEAR_RATIO = 1;
    private static final double CHASSIS_LENGTH = 7.078;
    private static final double ODO_XY_DISTANCE = 4.05; //x value
    private static final double ODO_YX_DISTANCE = 3.5; //Y value
    private static final double WINCH_RADIUS = 1;



    //debugging constants, not used very much

    File loggingFile = AppUtil.getInstance().getSettingsFile("Position.txt");
    File secondaryLoggingFile = AppUtil.getInstance().getSettingsFile("XSLAM.txt");
    File tertiaryloggingFile = AppUtil.getInstance().getSettingsFile("YSLAM.txt");
    File fourthLoggingFile = AppUtil.getInstance().getSettingsFile("Rotation.txt");
    File fifthLoggingFile = AppUtil.getInstance().getSettingsFile("vomega.txt");
    File sixthLoggingFile = AppUtil.getInstance().getSettingsFile("vy.txt");
    public String loggingString, secondaryLoggingString, tertiaryLoggingString, fourthLoggingString, fifthLoggingString, sixthLoggingString;
    private int iterator;
    //-2.641358450698 - (-2.641358450698 * 1.2)



    // set to true when thread is requested to shutdown
    private AtomicBoolean stop = new AtomicBoolean(false);
    // sensors run at 300 Hz
    // this is the length of that interval in milliseconds
    private long runInterval = (long)Math.round(1.0/50.0 * 1000.0);
    //1.0/300.0 * 1,000,000.0

    //hardware and timing constants, all of this is set up in the constructor
    private long elapsedTime, startingTime;
    private RobotPositionStateUpdater state;
    private final ExpansionHubMotor leftVertical, rightVertical, horizontal; //general odometry encoders, universal for each year
    private final ExpansionHubEx hub1;
    private RevBulkData data1, data2;
    private BNO055IMU imu;
    private double previousOuterArcLength = 0;
    private double previousInnerArcLength = 0;
    private double previousHorizontalArcLength = 0;
    private long minElapsedTime, maxElapsedTime;

    //Kalman filter parameters, the ones declared up here must also be tuned for EVERY ROBOT

    private static final double INCHES_TO_METERS = 0.0254; //-8.628937
        Transform2d cameraToRobot = new Transform2d(new Translation2d(0,0), new Rotation2d()); //-7.965 * INCHES_TO_METERS,  0 * INCHES_TO_METERS 0.8, -7.2
    private static T265Camera slamra;
    T265Camera.CameraUpdate currentSlamraPos;
    private Pose2d slamraStartingPose;
    private static double TAO = 0; //0.9 optimal
    private static final double MEASUREMENT_VARIANCE = 0.01; // 0.01 + 0.02? account more for odo variance
    private double previousEstimateUncertainty;
    Matrix previousOdoMat;
    Matrix previousIdealMat;
    Matrix previousVislamMat;

    //Game specific fields
    Servo odoWinch, secondaryOdoWinch;
    OdoState odoState;


    /**
     * Adjust the following to weigh the following out of your program
     *
     * High Frequency Sensor, TAO = 0
     * Low Frequency Sensor, TAO = 1
     *
     */





    //Non Kalman constructor, may make this an option later but for now this is deprecated
    /**
     * @param position in inches
     * @param globalRads in radians
     */
    @Deprecated
    public Localizer(HardwareMap hardwareMap, Vector2D position, double globalRads) {
        minElapsedTime = 0;
        maxElapsedTime = 0;
        hub1 = hardwareMap.get(ExpansionHubEx.class,"Control Hub");
        loggingString = "";
        secondaryLoggingString = "";
        //hub2 = hardwareMap.get(ExpansionHubEx.class,"Expansion Hub 2");
        // initialize hardware
        leftVertical = (ExpansionHubMotor)hardwareMap.dcMotor.get(Constants.LEFT_VERTICAL_ODOMETER_NAME);
        rightVertical = (ExpansionHubMotor)hardwareMap.dcMotor.get(Constants.RIGHT_VERTICAL_ODOMETER_NAME);
        horizontal = (ExpansionHubMotor)hardwareMap.dcMotor.get(Constants.HORIZONTAL_ODOMETER_NAME);


        // setup initial position;
        previousHorizontalArcLength = 0;
        previousInnerArcLength = 0;
        previousOuterArcLength = 0;
        startingTime = System.currentTimeMillis();
        state = new RobotPositionStateUpdater(position, new Vector2D(0,0), globalRads, 0);
        resetEncoders();

        odoState = OdoState.LOWERED;

    }

    //Kalman Constructor

    /**
     *
     * @param hardwareMap hardware interface we use, just passing in the opModes HardwareMap
     *                    field is always sufficient
     * @param position the starting position of the robot as a vector in inches
     * @param globalRads the starting orientation of the robot in radians
     * @param previousEstimateUncertainty the covariance of the kinematic models estimate,
     *                                    greater value means trusting the measured values more and
     *                                    smaller value means trusting the kinematic models estimate more
     */
    public Localizer(HardwareMap hardwareMap, Vector2D position, double globalRads, double previousEstimateUncertainty){

        if(slamra == null) {
            Debug.log("here");
            slamra = new T265Camera(cameraToRobot, 1.0, hardwareMap.appContext);
            currentSlamraPos = slamra.getLastReceivedCameraUpdate();
        }
        slamraStartingPose = new Pose2d(0 * INCHES_TO_METERS, 0 * INCHES_TO_METERS, new Rotation2d(globalRads));
        this.previousEstimateUncertainty = previousEstimateUncertainty; //this should always be a high value otherwise bad things will happen
        minElapsedTime = 0;
        maxElapsedTime = 0;
        hub1 = hardwareMap.get(ExpansionHubEx.class,"Control Hub");
        loggingString = "";
        secondaryLoggingString = "";
        //hub2 = hardwareMap.get(ExpansionHubEx.class,"Expansion Hub 2");
        // initialize hardware
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        leftVertical = (ExpansionHubMotor)hardwareMap.dcMotor.get(Constants.LEFT_VERTICAL_ODOMETER_NAME);
        rightVertical = (ExpansionHubMotor)hardwareMap.dcMotor.get(Constants.RIGHT_VERTICAL_ODOMETER_NAME);
        horizontal = (ExpansionHubMotor)hardwareMap.dcMotor.get(Constants.HORIZONTAL_ODOMETER_NAME);

        odoWinch = hardwareMap.servo.get("OdoWinch");
        secondaryOdoWinch = hardwareMap.servo.get("SecondaryOdoWinch");

        // setup initial position;
        previousHorizontalArcLength = 0;
        previousInnerArcLength = 0;
        previousOuterArcLength = 0;

        previousVislamMat = new Matrix(6,1);
        previousIdealMat = new Matrix(6,1);
        startingTime = System.currentTimeMillis();
        state = new RobotPositionStateUpdater(position, new Vector2D(0,0), globalRads, 0);

        double[][] previousOdoArray = {
                {state.getCurrentState().getPosition().getX()},
                {state.getCurrentState().getPosition().getY()},
                {state.getCurrentState().getRotation()},
                {0},
                {0}, //this whole class assumes constant velocity and it is fair to assume the robot starts still
                {0}
        };
        previousOdoMat = new Matrix(previousOdoArray);

        //lowerOdo();
        resetEncoders();
        iterator = 1;
        odoState = OdoState.LOWERED;
    }



    public void stopThread() {
        if(slamra != null) {
            Debug.log("end");
            slamra.stop();
        }
        this.stop.set(true);
    }
    @Override
    public void run() {
        // make sure we reset our accounting of start times
        state.resetUpdateTime();
        startingTime = System.currentTimeMillis();
        if(slamra != null) {
            slamra.start();
            currentSlamraPos = slamra.getLastReceivedCameraUpdate();
            slamra.setPose(slamraStartingPose);
        }
        // max speed 300 Hz)
        while (!stop.get()) {
            long millis = System.currentTimeMillis();
            //update();
           //updateKalman();
            matUpdate();
            long runtime = System.currentTimeMillis() - millis;

            if (runtime > runInterval) {
                // this is very bad
                // todo break here.
                minElapsedTime++;
                runtime=runInterval;
            }
            maxElapsedTime++;
//            loggingString += (runInterval - runtime) +"\n";
//            ReadWriteFile.writeFile(loggingFile, loggingString);
            try {
                // cast should be fine here since we're likely dealing with only
                // a few milliseconds
                sleep(runInterval - runtime);
            } catch (InterruptedException e) {
                // this probably isn't bad.
                e.printStackTrace();
            }
        }
    }

    //initializing the odo, not to be confused with zeroing it
    private void resetEncoders() {
        leftVertical.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightVertical.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        horizontal.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftVertical.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightVertical.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        horizontal.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //horizontal.setDirection(DcMotorSimple.Direction.REVERSE);
        //leftVertical.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void liftOdo(){
        odoWinch.setPosition(1.0);
        secondaryOdoWinch.setPosition(0.25);
        odoState = OdoState.RAISED;
    }

    public void lowerOdo(){
        odoWinch.setPosition(0);
        secondaryOdoWinch.setPosition(0);
        odoState = OdoState.LOWERED;
    }

    private enum OdoState{
        RAISED, LOWERED
    }

    //zeroing the odo
    public void resetOdometersTravelling(){
        leftVertical.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightVertical.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        horizontal.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        previousHorizontalArcLength = 0;
        previousInnerArcLength = 0;
        previousOuterArcLength = 0;
    }

    //exposes the state to exterior classes
    public RobotPositionStateUpdater.RobotPositionState getCurrentState() {
        return state.getCurrentState();
    }

    //see top of class for formalized proof of the math
    @Deprecated //use updateKalman instead
    private synchronized void update() {
        // read sensor data
        data1 = hub1.getBulkInputData();


        double innerArcLength = encoderTicksToInches(data1.getMotorCurrentPosition(leftVertical));
        // encoder orientation is the same, which means they generate opposite rotation signals
        double outerArcLength =  encoderTicksToInches(data1.getMotorCurrentPosition(rightVertical));
        double horizontalArcLength = encoderTicksToInches(data1.getMotorCurrentPosition(horizontal));

        double leftVerticalVelocity = encoderTicksToInches(data1.getMotorVelocity(leftVertical));
        double rightVerticalVelocity = encoderTicksToInches(data1.getMotorVelocity(rightVertical));
        double horizontalVelocity = encoderTicksToInches(data1.getMotorVelocity(horizontal));

        // calculate positions
        double deltaInnerArcLength = innerArcLength - previousInnerArcLength;
        double deltaOuterArcLength = outerArcLength - previousOuterArcLength;
        double deltaHorizontalArcLength = horizontalArcLength - previousHorizontalArcLength;

        double arcLength = (deltaInnerArcLength + deltaOuterArcLength) / 2.0;
        double deltaVerticalDiff = (deltaInnerArcLength - deltaOuterArcLength) / 2.0;

//(deltaOuterArcLength - deltaInnerArcLength)
        // CHASSIS_LENGTH is the diamater of the circle.
        // phi is arclength divided by radius for small phi
        double phi =  (2.0 * deltaVerticalDiff) / (CHASSIS_LENGTH);
        double hypotenuse;



        // When phi is small, the full formula is numerically unstable.
        // for small phi, sin(phi) = phi and cos(phi) = 1
        // thus small phi, hypotense = arcLength
        if(abs(phi) < 0.0001){
            hypotenuse = arcLength;
        }else{
            hypotenuse = (arcLength * sin(phi)) / (phi * cos(phi / 2.0));
        }

        double horizontalDelta = deltaHorizontalArcLength - (phi * ODO_XY_DISTANCE);
        double verticalDelta = hypotenuse * cos(phi/2.0)  + deltaVerticalDiff - (phi *ODO_YX_DISTANCE);


        // calculate velocities
        // a difference in velocity will be due to rotation.
        // however since both encoders count this difference, this is double counted
        // So arc length of rotation divided by the radius gives us the rotational velocity
        // the factors of two cancel!
        double omega = (leftVerticalVelocity - rightVerticalVelocity)/CHASSIS_LENGTH;
        double deltaVy = (leftVerticalVelocity + rightVerticalVelocity)/2.0;
        double deltaVx = horizontalVelocity;
        //Debug.log(horizontalDelta);
        state.updateDelta(horizontalDelta, verticalDelta, phi, deltaVx, deltaVy, omega);
        previousInnerArcLength = innerArcLength;
        previousOuterArcLength = outerArcLength;
        previousHorizontalArcLength = horizontalArcLength;
        elapsedTime = System.currentTimeMillis() - startingTime;
//        loggingString +=  hypotenuse  + "\n";
//        loggingString +=  arcLength  + "\n";
//        loggingString += deltaVerticalDiff + "\n";
//        loggingString += phi + "\n";
//        loggingString += deltaInnerArcLength + "\n";
//        loggingString += deltaOuterArcLength + "\n";
//        loggingString += deltaHorizontalArcLength + "\n";
//        loggingString += "\n";
//        ReadWriteFile.writeFile(loggingFile, loggingString);
        //AbstractOpMode.currentOpMode().telemetry.addData("AL:", arcLength);
        //AbstractOpMode.currentOpMode().telemetry.addData("DVD:", deltaVerticalDiff);
//        AbstractOpMode.currentOpMode().telemetry.addData("", this::getCurrentState);
//        AbstractOpMode.currentOpMode().telemetry.update();
    }




    //updating a cycle using the Kalman Filter, a method which takes 3 measurements, Odometry, VISLAM
    //and a third measurement using physics to predict where the robot is (we call this the kinematic model)
    private synchronized void updateKalman(){
        // read sensor data
        data1 = hub1.getBulkInputData();
        currentSlamraPos = slamra.getLastReceivedCameraUpdate();
        RobotPositionStateUpdater.RobotPositionState currentState = getCurrentState();

        int leftVerticalTics = data1.getMotorCurrentPosition(leftVertical);
        int rightVerticalTics = data1.getMotorCurrentPosition(rightVertical);
        int horizontalTics = data1.getMotorCurrentPosition(horizontal);

        double innerArcLength = encoderTicksToInches(leftVerticalTics);
        // encoder orientation is the same, which means they generate opposite rotation signals
        double outerArcLength =  -encoderTicksToInches(rightVerticalTics);
        double horizontalArcLength = -encoderTicksToInches(horizontalTics);

//        double leftVerticalVelocity = encoderTicksToInches(data1.getMotorVelocity(leftVertical));
//        double rightVerticalVelocity = -encoderTicksToInches(data1.getMotorVelocity(rightVertical));
//        double horizontalVelocity = -encoderTicksToInches(data1.getMotorVelocity(horizontal));

        // calculate positions
        double deltaInnerArcLength = innerArcLength - previousInnerArcLength;
        double deltaOuterArcLength = outerArcLength - previousOuterArcLength;
        double deltaHorizontalArcLength = horizontalArcLength - previousHorizontalArcLength;

        double leftVerticalVelocity =deltaInnerArcLength / 0.02;
        double rightVerticalVelocity = deltaOuterArcLength / 0.02;
        double horizontalVelocity = deltaHorizontalArcLength/ 0.02;

        double arcLength = (deltaInnerArcLength + deltaOuterArcLength) / 2.0;
        double deltaVerticalDiff = (deltaInnerArcLength - deltaOuterArcLength) / 2.0;


        // CHASSIS_LENGTH is the diamater of the circle.
        // phi is arclength divided by radius for small phi
        double phi =  (2.0 * deltaVerticalDiff) / (CHASSIS_LENGTH);
        double hypotenuse;

        double rotation = previousOdoMat.getValue(2,0);

        //loggingString += currentState.getPosition().getX() + " , " + currentState.getPosition().getY() + " , " + currentState.getRotation() + "\n";
        // When phi is small, the full formula is numerically unstable.
        // for small phi, sin(phi) = phi and cos(phi) = 1
        // thus small phi, hypotense = arcLength
        if(abs(phi) < 0.1){ //0.001
            hypotenuse = arcLength;
        }else{
            hypotenuse = (arcLength * sin(rotation + phi)) / (phi * cos(rotation + phi / 2.0));
        }

        double horizontalDelta = deltaHorizontalArcLength  + hypotenuse * sin(rotation +phi); // - (phi * ODO_XY_DISTANCE) X
        double verticalDelta = hypotenuse * cos(rotation + phi) - (phi * ODO_YX_DISTANCE); //y


        // calculate velocities
        // a difference in velocity will be due to rotation.
        // however since both encoders count this difference, this is double counted
        // So arc length of rotation divided by the radius gives us the rotational velocity
        // the factors of two cancel!
        double omega = (leftVerticalVelocity - rightVerticalVelocity)/CHASSIS_LENGTH;
        double deltaVy = (leftVerticalVelocity + rightVerticalVelocity)/2.0;
        double deltaVx = horizontalVelocity;

//        if(odoState == OdoState.RAISED){
//            TAO = 0;
//        }else{
//            TAO = 1;
//        }

        //scaling the deltas to make the Odometry its own independent measurement
        horizontalDelta *= TAO;
        verticalDelta *= TAO;
        phi *= TAO;
        omega *= TAO;
        deltaVx *= TAO;
        deltaVy *= TAO;


        //matrix declarations
        final double robotRadius =7.7;
        Pose2d slamraEstimatePose = currentSlamraPos.pose;
        double slamx = -slamraEstimatePose.getY() / INCHES_TO_METERS;
        double slamy = 7.7 + (slamraEstimatePose.getX() / INCHES_TO_METERS);
        double slamRotation = currentSlamraPos.pose.getRotation().getRadians();
        double trueX = slamx + sin(slamRotation) * robotRadius;
        double trueY = slamy - (cos(slamRotation) * robotRadius);

        double[][] vislamMat = {
                {trueX},
                {trueY},
                {-slamraEstimatePose.getRotation().getRadians()},
                {-currentSlamraPos.velocity.vyMetersPerSecond / INCHES_TO_METERS},
                {currentSlamraPos.velocity.vxMetersPerSecond / INCHES_TO_METERS},
                {currentSlamraPos.velocity.omegaRadiansPerSecond}
        };
//        if(abs(vislamMat[3][0]) < 0.2){
//            vislamMat[3][0] = 0;
//        }
//        if(abs(vislamMat[4][0]) < 0.2){
//            vislamMat[4][0] = 0;
//        }
//        if(abs(vislamMat[5][0]) < 0.2){
//            vislamMat[5][0] = 0;
//        }
        Matrix slamraEstimate = new Matrix(vislamMat);

        double[][] odoMat = {
                {previousOdoMat.getValue(0,0) + horizontalDelta},
                {previousOdoMat.getValue(1,0) + verticalDelta},
                {previousOdoMat.getValue(2,0) + phi},
                {deltaVx},
                {deltaVy},
                {omega}
        };
        Matrix odoEstimate = new Matrix(odoMat);

        double[][] currentStateMat = {
                {currentState.getPosition().getX()},
                {currentState.getPosition().getY()},
                {currentState.getRotation()},
                {currentState.getVelocity().getX()},
                {currentState.getVelocity().getY()},
                {currentState.getAngularVelocity()}
        };
        Matrix currentStateEstimate = new Matrix(currentStateMat);

        elapsedTime = System.currentTimeMillis() - startingTime;

        double[][] deltaMat = {
                {currentState.getVelocity().getX() * elapsedTime / 1000.0},
                {currentState.getVelocity().getY() * elapsedTime / 1000.0},
                {currentState.getAngularVelocity() * elapsedTime / 1000.0},
                {0},
                {0},
                {0}
        };


        Matrix kinematicModel = new Matrix(deltaMat);
        Matrix idealEstimate = kinematicModel.add(currentStateEstimate);

        //update this here to keep odo estimate truly independent of everything else,
        //if one system starts drifting we dont want the covariance of the odo to suffer
        double[][] previousOdoArray = {
                {odoEstimate.getValue(0,0)},
                {odoEstimate.getValue(1,0)},
                {odoEstimate.getValue(2,0)},
                {odoEstimate.getValue(3,0)},
                {odoEstimate.getValue(4,0)},
                {odoEstimate.getValue(5,0)},
        };
            previousOdoMat = new Matrix(previousOdoArray);

        slamraEstimate.multiply(1.0-TAO);
        Matrix complementaryStateEstimtate = odoEstimate.add(slamraEstimate); //measured state, Z

        double kalmanGain = previousEstimateUncertainty / (previousEstimateUncertainty + MEASUREMENT_VARIANCE);

        kalmanGain = 1;

        double currentEstimateUncertainty = (1 - kalmanGain) * previousEstimateUncertainty;


        idealEstimate.multiply(1 - kalmanGain);
        //multiplying only the distance components since we assume constant velocity
        //the kinematic model assumes constant velocity meaning that the values was
        //being greatly diminished causing overacceleration
        complementaryStateEstimtate.setValue(0,0, complementaryStateEstimtate.getValue(0,0) * kalmanGain);
        complementaryStateEstimtate.setValue(1,0, complementaryStateEstimtate.getValue(1,0) * kalmanGain);
        complementaryStateEstimtate.setValue(2,0, complementaryStateEstimtate.getValue(2,0) * kalmanGain);
        Matrix finalStateEstimate = complementaryStateEstimtate.add(idealEstimate);


        double dx = finalStateEstimate.getValue(0,0);
        double dy = finalStateEstimate.getValue(1,0);
        double dphi = finalStateEstimate.getValue(2,0);
        double dvx = complementaryStateEstimtate.getValue(3,0);
        double dvy = complementaryStateEstimtate.getValue(4,0);
        double domega = complementaryStateEstimtate.getValue(5,0);


        state.updateState(dx, dy, dphi, dvx, dvy, domega);

        loggingString += state.getCurrentState().getPosition().loggerToString()+ "\n";
        secondaryLoggingString += iterator + "," +  state.getCurrentState().getPosition().getX() + "\n";
        tertiaryLoggingString += iterator+ "," +  state.getCurrentState().getPosition().getY() + "\n";
//        fourthLoggingString += iterator + "," + rotation + "\n";
//        fifthLoggingString += iterator + "," + domega + "\n";
//        sixthLoggingString += iterator+ "," + dvy + "\n";
        startingTime = System.currentTimeMillis();
        previousInnerArcLength = innerArcLength;
        previousOuterArcLength = outerArcLength;
        previousHorizontalArcLength = horizontalArcLength;
        previousEstimateUncertainty = currentEstimateUncertainty;
        iterator++;
    }

    DecompositionSolver solver;

    ArrayList<Pose> wheelPoses;
    double[] lastWheelPositions;
    private Pose odoEstimate;
    private Pose poseVelocity;



    public Localizer(Pose start, HardwareMap hardwareMap){
        hub1 = hardwareMap.get(ExpansionHubEx.class,"Control Hub");
        loggingString = "";
        secondaryLoggingString = "";
        state = new RobotPositionStateUpdater();
        if(slamra == null) {
            Debug.log("here");
            slamra = new T265Camera(cameraToRobot, 1.0, hardwareMap.appContext);
            currentSlamraPos = slamra.getLastReceivedCameraUpdate();
        }
        slamraStartingPose = new Pose2d(0 * INCHES_TO_METERS, 0 * INCHES_TO_METERS, new Rotation2d(start.heading));

        //hub2 = hardwareMap.get(ExpansionHubEx.class,"Expansion Hub 2");
        // initialize hardware
        leftVertical = (ExpansionHubMotor)hardwareMap.dcMotor.get(Constants.LEFT_VERTICAL_ODOMETER_NAME);
        rightVertical = (ExpansionHubMotor)hardwareMap.dcMotor.get(Constants.RIGHT_VERTICAL_ODOMETER_NAME);
        horizontal = (ExpansionHubMotor)hardwareMap.dcMotor.get(Constants.HORIZONTAL_ODOMETER_NAME);
        odoWinch = hardwareMap.servo.get("OdoWinch");
        secondaryOdoWinch = hardwareMap.servo.get("SecondaryOdoWinch");


        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.RADIANS;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);


        Array2DRowRealMatrix inverseMatrix = new Array2DRowRealMatrix(3, 3);


        wheelPoses = new ArrayList<>();
        wheelPoses.add(new Pose(4.01885,-6.471937, Math.toRadians(180))); //LV // // -3.91885,-6.471937
        wheelPoses.add(new Pose(4.5222, 0.10685, Math.toRadians(90))); //H //-4.5222, 0.10685
        //wheelPoses.add(new Vector2D(4.18685,-6.336937)); //RV


        for (int i = 0; i < 2; i++) {
            Pose position = wheelPoses.get(i);
            double x = cos(position.heading);
            double y = sin(position.heading);

            double val = position.x * y - position.y * x;

            inverseMatrix.setEntry(i, 0, x);
            inverseMatrix.setEntry(i, 1, y);
            inverseMatrix.setEntry(i, 2, val);
//            Debug.log(positionVector.getX() * orientationVector.getY());
//            Debug.log(positionVector.getY() * orientationVector.getX());
        }

        inverseMatrix.setEntry(2,2,1.0);



        solver = new LUDecomposition(inverseMatrix).getSolver();
        Debug.log(solver.isNonSingular());
        lastWheelPositions = new double[]{0,0,0};
        odoEstimate = start.clone();
        poseVelocity = new Pose(0,0,0);
        previousHeading = 0;
        odoState = OdoState.LOWERED;


        resetEncoders();
        iterator = 1;

    }

    public double directionToHeading(double direction){
        return Math.PI / 2.0 - direction;
    }





    double previousHeading;
    private static final double X_SCALAR = 0.1;
    private static final double Y_SCALAR = 0.1;
    private synchronized void matUpdate() {
        data1 = hub1.getBulkInputData();
        currentSlamraPos = slamra.getLastReceivedCameraUpdate();
        double heading = (imu.getAngularOrientation().firstAngle);
        double dHeading = heading - previousHeading;



        double[] wheelPositions = new double[]{encoderTicksToInches(data1.getMotorCurrentPosition(leftVertical)),
                encoderTicksToInches(data1.getMotorCurrentPosition(horizontal)),
                -encoderTicksToInches(data1.getMotorCurrentPosition(rightVertical)),
                };
        double[] wheelDeltas = new double[]{wheelPositions[0] - lastWheelPositions[0], wheelPositions[1] - lastWheelPositions[1], dHeading};
             Pose robotPoseDelta = calculatePoseDelta(wheelDeltas);
             odoEstimate = relativeOdometryUpdate(robotPoseDelta);
        double[] wheelVelocities = new double[]{wheelDeltas [0]/ 0.02, wheelDeltas[1] / 0.02, wheelDeltas[2] / 0.02};

        if (wheelVelocities != null) {
            poseVelocity = calculatePoseDelta(wheelVelocities);
        }

        final double robotRadius =7.7;
        Pose2d slamraEstimatePose = currentSlamraPos.pose;
        double slamx = -slamraEstimatePose.getY() / INCHES_TO_METERS;
        double slamy = 7.7 + (slamraEstimatePose.getX() / INCHES_TO_METERS);
        double slamRotation = currentSlamraPos.pose.getRotation().getRadians();
        double trueX = slamx + sin(slamRotation) * robotRadius;
        double trueY = slamy - (cos(slamRotation) * robotRadius);


        if(odoState == OdoState.LOWERED){
            TAO = 0.9;
        }else{
            TAO = 0;
        }

        double[][] vislamMat = {
                {trueX},
                {trueY},
                {angleWrap(slamraEstimatePose.getRotation().getRadians())},
                {-currentSlamraPos.velocity.vyMetersPerSecond / INCHES_TO_METERS},
                {currentSlamraPos.velocity.vxMetersPerSecond / INCHES_TO_METERS},
                {currentSlamraPos.velocity.omegaRadiansPerSecond}
        };
//        if(abs(vislamMat[3][0]) < 0.2){
//            vislamMat[3][0] = 0;
//        }
//        if(abs(vislamMat[4][0]) < 0.2){
//            vislamMat[4][0] = 0;
//        }
//        if(abs(vislamMat[5][0]) < 0.2){
//            vislamMat[5][0] = 0;
//        }
        Matrix slamraEstimate = new Matrix(vislamMat);

        double[][] odoMat = {
                {odoEstimate.x},
                {odoEstimate.y},
                {odoEstimate.heading},
                {poseVelocity.x},
                {poseVelocity.y},
                {poseVelocity.heading}
        };
        Matrix odoMatrix = new Matrix(odoMat);

        odoMatrix.multiply(TAO);

        slamraEstimate.multiply(1.0-TAO);
        Matrix complementaryStateEstimtate = odoMatrix.add(slamraEstimate); //measured state, Z

        state.updateState(complementaryStateEstimtate.getValue(0,0),
                complementaryStateEstimtate.getValue(1,0),
                complementaryStateEstimtate.getValue(2,0),
                complementaryStateEstimtate.getValue(3,0),
                complementaryStateEstimtate.getValue(4,0),
                complementaryStateEstimtate.getValue(5,0));

        previousHeading = heading;

        lastWheelPositions = new double[]{wheelPositions[0], wheelPositions[1], wheelPositions[2]};


        loggingString += odoEstimate.x + "," + odoEstimate.y +  "\n";
        secondaryLoggingString += iterator + "," +  odoEstimate.x + "\n";
        tertiaryLoggingString += iterator+ "," +  odoEstimate.y + "\n";
        iterator++;
        }

        private double angleWrap(double angle) {
            while(angle > PI){
                angle -= 2 * PI;
            }
            while(angle < -PI){
                angle += 2 * PI;
            }
            return angle;
        }

    private static final double EPSILON = 1e-6;

    private Pose relativeOdometryUpdate(Pose robotPoseDelta) {
        double dtheta = robotPoseDelta.heading;
        double sineTerm, cosTerm;

        if (approxEquals(dtheta, 0)) {
            sineTerm = 1.0 - dtheta * dtheta / 6.0;
            cosTerm = dtheta / 2.0;
        } else {
            sineTerm = Math.sin(dtheta) / dtheta;
            cosTerm = (1 - Math.cos(dtheta)) / dtheta;
        }

        Point fieldPositionDelta = new Point(
                sineTerm * robotPoseDelta.x - cosTerm * robotPoseDelta.y,
                cosTerm * robotPoseDelta.x + sineTerm * robotPoseDelta.y
        );

        Pose fieldPoseDelta = new Pose(fieldPositionDelta.rotated(odoEstimate.heading), robotPoseDelta.heading);



        Pose fieldPose = new Pose(odoEstimate.x,odoEstimate.y, odoEstimate.heading);

        return fieldPose.add(fieldPoseDelta);
    }

    public static boolean approxEquals(double d1, double d2) {
        if (Double.isInfinite(d1)) {
            // Infinity - infinity is NaN, so we need a special case
            return d1 == d2;
        } else {
            return Math.abs(d1 - d2) < EPSILON;
        }
    }

    private Pose calculatePoseDelta(double[] wheelDeltas) {
        RealMatrix m = MatrixUtils.createRealMatrix(new double[][]{wheelDeltas});
        RealMatrix rawPoseDelta = solver.solve(m.transpose());
        return new Pose(
                rawPoseDelta.getEntry(0, 0),
                rawPoseDelta.getEntry(1, 0),
                rawPoseDelta.getEntry(2, 0));

    }


    public Pose getOdoEstimate(){
        return odoEstimate;
    }

    public Pose getPoseVelocity(){
        return poseVelocity;
    }

    public int getIterator(){
        return iterator;
    }





    /*
     Array2DRowRealMatrix inverseMatrix = new Array2DRowRealMatrix(3, 3);
        ArrayList<Vector3D> wheelPoses = new ArrayList<Vector3D>();
        Vector3D LV_POSE = new Vector3D(0,0, 0); //x, y, angle from center
        Vector3D RV_POSE = new Vector3D(0,0, Math.toRadians(180));
        Vector3D H_POSE = new Vector3D(0,0, Math.toRadians(90));
        wheelPoses.add(LV_POSE);
        wheelPoses.add(RV_POSE);
        wheelPoses.add(H_POSE);

        for(Vector3D wheel: wheelPoses){
            double wheelXOffset = Math.cos(wheel.getZ());
            double wheelYOffset = Math.sin(wheel.getZ());

        }
     */

    public void manualPositionWrite(RobotPositionStateUpdater.RobotPositionState newState){
        state.updateState(newState.getPosition().getX(), newState.getPosition().getY(), newState.getRotation(), newState.getVelocity().getX(), newState.getVelocity().getY(), newState.getAngularVelocity());
    }

    private double linearSlideEncoderTicksToInches(int motorCurrentPosition) {
        return motorCurrentPosition / TICKS_PER_REV;
    }


    public long getMinElapsedTime(){
        return minElapsedTime;
    }

    public long getMaxElapsedTime(){
        return maxElapsedTime;
    }

    public static double encoderTicksToInches(int ticks) {
        return WHEEL_DIAMETER * PI * GEAR_RATIO * ticks / TICKS_PER_REV;
    }
    public static int inchesToEncoderTicks(double inches){
        return (int)((TICKS_PER_REV / (WHEEL_DIAMETER  * PI * GEAR_RATIO)) * inches);
    }

    public double getLeftVerticalOdometerPosition(){
        return encoderTicksToInches(leftVertical.getCurrentPosition());
    }

    public double getLeftVerticalOdometerVelocity(){
        return leftVertical.getVelocity();
    }

    public double getRightVerticalOdometerVelocity(){
        return rightVertical.getVelocity();
    }

    public double getHorizontalOdometerVelocity(){
        return horizontal.getVelocity();
    }

    public double getRightVerticalOdometerPosition(){
        return -encoderTicksToInches(rightVertical.getCurrentPosition());
    }

    public double getHorizontalOdometerPosition(){
        return -encoderTicksToInches(horizontal.getCurrentPosition());
    }

    public void writeLoggerToFile(){
        try {
            PrintStream ps = new PrintStream(loggingFile);
            PrintStream pstwo = new PrintStream(secondaryLoggingFile);
            PrintStream psThree = new PrintStream(tertiaryloggingFile);
            PrintStream psfour = new PrintStream(fourthLoggingFile);
            PrintStream psfive = new PrintStream(fifthLoggingFile);
            PrintStream psSix = new PrintStream(sixthLoggingFile);
            ps.println(loggingString);
            pstwo.println(secondaryLoggingString);
            psThree.println(tertiaryLoggingString);
            psfour.println(fourthLoggingString);
            psfive.println(fifthLoggingString);
            psSix.println(sixthLoggingString);
        } catch (FileNotFoundException e) {
            e.printStackTrace();
        }

    }
}









//  Matrix shit, saving the start of my unsimplified proof of the math
//        double[][] stateTransistionMat = {
//                {currentState.getVelocity().getX() * elapsedTime * 1000, 0, 0, 0, 0, 0},
//                {0, currentState.getVelocity().getY() * elapsedTime * 1000, 0, 0, 0, 0},
//                {0, 0, currentState.getAngularVelocity() * elapsedTime * 1000, 0, 0, 0},
//                {0, 0, 0, currentState.getVelocity().getX(), 0, 0},
//                {0, 0, 0, 0, currentState.getVelocity().getY(), 0},
//                {0, 0, 0, 0, 0, currentState.getAngularVelocity()}
//        };
//
//        Matrix stateTransistionMatrix = new Matrix(stateTransistionMat);
//
//        //we have an estimate according to kinematics (assumes constant acceleration)
//        //Matrix idealStateEstimate = currentStateEstimate.add(KinematicMotionModel);
//        Matrix transposedStateTransitionMatrix = stateTransistionMatrix.transpose();
//        Matrix controlMatrix;
//
//        Matrix estimateUncertianty = stateTransistionMatrix.multiply(previousProcessNoise);
//        estimateUncertianty = estimateUncertianty.multiply(transposedStateTransitionMatrix);
