package teamcode.common;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Transform2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.util.ReadWriteFile;
import com.spartronics4915.lib.T265Camera;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.openftc.revextensions2.ExpansionHubEx;
import org.openftc.revextensions2.ExpansionHubMotor;
import org.openftc.revextensions2.RevBulkData;

import java.io.File;
import java.util.concurrent.atomic.AtomicBoolean;

import static java.lang.Math.*;

public class Localizer extends Thread {
    //TODO before reading this file please note the static import of the math class,
    // odds are if you see a math function it is from that and not a constatnt/method I created
    //https://docs.google.com/document/d/1JQuU2M--rVFEa9ApKkOcc0CalsBxKjH-l0PQLRW7F3c/edit?usp=sharing proof behind the math

    //odometry wheel constants, MUST BE CALIBRATED FOR EACH ROBOT
    private static final double TICKS_PER_REV = 8192;
    private static final double WHEEL_RADIUS = 1.421; //1.181 for 60 mm, 1.417 for 72mm
    private static final double GEAR_RATIO = 1;
    private static final double CHASSIS_LENGTH = 12.4; //new bot + 2.83465
    private static final double ODO_XY_DISTANCE = 1.925;
    private static final double WINCH_RADIUS = 1;

    //debugging constants, not used very much
    File loggingFile = AppUtil.getInstance().getSettingsFile("LinearEncoderReadings.txt");
    public String loggingString;
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
    private final BNO055IMU imu;
    private double previousOuterArcLength = 0;
    private double previousInnerArcLength = 0;
    private double previousHorizontalArcLength = 0;
    private long minElapsedTime, maxElapsedTime;

    //Kalman filter parameters, the ones declared up here must also be tuned for EVERY ROBOT
    Transform2d cameraToRobot = new Transform2d(new Translation2d(9 * 0.0254, 0), new Rotation2d());
    private static T265Camera slamra;
    T265Camera.CameraUpdate currentSlamraPos;
    private Pose2d slamraStartingPose;
    private static final double TAO = 0;
    private static final double MEASUREMENT_VARIANCE = 0.01; // 0.01 + 0.02? account more for odo variance
    private double previousEstimateUncertainty;
    Matrix previousOdoMat;
    Matrix previousIdealMat;
    Matrix previousVislamMat;

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
        //hub2 = hardwareMap.get(ExpansionHubEx.class,"Expansion Hub 2");
        // initialize hardware
        imu = hardwareMap.get(BNO055IMU.class, "imu");
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
            slamra = new T265Camera(cameraToRobot, 1.0, hardwareMap.appContext);
            currentSlamraPos = slamra.getLastReceivedCameraUpdate();
        }
        slamraStartingPose = new Pose2d(position.getY() * 0.0254, position.getX() * 0.0254, new Rotation2d(globalRads));
        this.previousEstimateUncertainty = previousEstimateUncertainty; //this should always be a high value otherwise bad things will happen
        minElapsedTime = 0;
        maxElapsedTime = 0;
        hub1 = hardwareMap.get(ExpansionHubEx.class,"Control Hub");
        loggingString = "";
        //hub2 = hardwareMap.get(ExpansionHubEx.class,"Expansion Hub 2");
        // initialize hardware
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        leftVertical = (ExpansionHubMotor)hardwareMap.dcMotor.get(Constants.LEFT_VERTICAL_ODOMETER_NAME);
        rightVertical = (ExpansionHubMotor)hardwareMap.dcMotor.get(Constants.RIGHT_VERTICAL_ODOMETER_NAME);
        horizontal = (ExpansionHubMotor)hardwareMap.dcMotor.get(Constants.HORIZONTAL_ODOMETER_NAME);
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
        previousOdoMat = new Matrix(6,1);

        slamra.start();
        resetEncoders();
    }



    public void stopThread() {
        if(slamra != null) {
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
            currentSlamraPos = slamra.getLastReceivedCameraUpdate();
            slamra.setPose(slamraStartingPose);
        }
        // max speed 300 Hz)
        while (!stop.get()) {
            long millis = System.currentTimeMillis();
            //update();
            updateKalman();
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
        double outerArcLength = - encoderTicksToInches(data1.getMotorCurrentPosition(rightVertical));
        double horizontalArcLength = -encoderTicksToInches(data1.getMotorCurrentPosition(horizontal));

        double leftVerticalVelocity = encoderTicksToInches(data1.getMotorVelocity(leftVertical));
        double rightVerticalVelocity = encoderTicksToInches(data1.getMotorVelocity(rightVertical));
        double horizontalVelocity = -encoderTicksToInches(data1.getMotorVelocity(horizontal));

        // calculate positions
        double deltaInnerArcLength = innerArcLength - previousInnerArcLength;
        double deltaOuterArcLength = outerArcLength - previousOuterArcLength;
        double deltaHorizontalArcLength = horizontalArcLength - previousHorizontalArcLength;

        double arcLength = (deltaInnerArcLength + deltaOuterArcLength) / 2.0;
        double deltaVerticalDiff = (deltaInnerArcLength - deltaOuterArcLength) / 2.0;

//(deltaOuterArcLength - deltaInnerArcLength)
        // CHASSIS_LENGTH is the diamater of the circle.
        // phi is arclength divided by radius for small phi
        double phi =  (2.0 * arcLength) / (CHASSIS_LENGTH);
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
        double verticalDelta = hypotenuse * cos(phi/2.0)  + deltaVerticalDiff - (phi * CHASSIS_LENGTH / 2.0);


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
//        NormalizedRGBA houseRGBA = houseSensor.getNormalizedColors();
        RobotPositionStateUpdater.RobotPositionState currentState = getCurrentState();

        double innerArcLength = encoderTicksToInches(data1.getMotorCurrentPosition(leftVertical));
        // encoder orientation is the same, which means they generate opposite rotation signals
        double outerArcLength = - encoderTicksToInches(data1.getMotorCurrentPosition(rightVertical));
        double horizontalArcLength = -encoderTicksToInches(data1.getMotorCurrentPosition(horizontal));

        double leftVerticalVelocity = encoderTicksToInches(data1.getMotorVelocity(leftVertical));
        double rightVerticalVelocity = encoderTicksToInches(data1.getMotorVelocity(rightVertical));
        double horizontalVelocity = -encoderTicksToInches(data1.getMotorVelocity(horizontal));

        // calculate positions
        double deltaInnerArcLength = innerArcLength - previousInnerArcLength;
        double deltaOuterArcLength = outerArcLength - previousOuterArcLength;
        double deltaHorizontalArcLength = horizontalArcLength - previousHorizontalArcLength;

        double arcLength = (deltaInnerArcLength + deltaOuterArcLength) / 2.0;
        double deltaVerticalDiff = (deltaInnerArcLength - deltaOuterArcLength) / 2.0;


        // CHASSIS_LENGTH is the diamater of the circle.
        // phi is arclength divided by radius for small phi
        double phi =  (2.0 * arcLength) / (CHASSIS_LENGTH);
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
        double verticalDelta = hypotenuse * cos(phi/2.0)  + deltaVerticalDiff - (phi * (CHASSIS_LENGTH)/ 2.0);


        // calculate velocities
        // a difference in velocity will be due to rotation.
        // however since both encoders count this difference, this is double counted
        // So arc length of rotation divided by the radius gives us the rotational velocity
        // the factors of two cancel!
        double omega = (leftVerticalVelocity - rightVerticalVelocity)/CHASSIS_LENGTH;
        double deltaVy = (leftVerticalVelocity + rightVerticalVelocity)/2.0;
        double deltaVx = horizontalVelocity;

        //scaling the deltas to make the Odometry its own independent measurement
        horizontalDelta *= TAO;
        verticalDelta *= TAO;
        phi *= TAO;
        omega *= TAO;
        deltaVx *= TAO;
        deltaVy *= TAO;


        //matrix declarations
        Pose2d slamraEstimatePose = currentSlamraPos.pose;
        double[][] vislamMat = {
                {-slamraEstimatePose.getY() / 0.0254},
                {slamraEstimatePose.getX() / 0.0254},
                {slamraEstimatePose.getRotation().getRadians()},
                {-currentSlamraPos.velocity.vyMetersPerSecond / 0.0254},
                {currentSlamraPos.velocity.vxMetersPerSecond / 0.0254},
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
        startingTime = System.currentTimeMillis();
        previousInnerArcLength = innerArcLength;
        previousOuterArcLength = outerArcLength;
        previousHorizontalArcLength = horizontalArcLength;
        previousEstimateUncertainty = currentEstimateUncertainty;
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
        return WHEEL_RADIUS * 2 * PI * GEAR_RATIO * ticks / TICKS_PER_REV;
    }
    public static int inchesToEncoderTicks(double inches){
        return (int)((TICKS_PER_REV / (WHEEL_RADIUS * 2 * PI * GEAR_RATIO)) * inches);
    }

    public int getLeftVerticalOdometerPosition(){
        return leftVertical.getCurrentPosition();
    }

    public int getRightVerticalOdometerPosition(){
        return rightVertical.getCurrentPosition();
    }

    public int getHorizontalOdometerPosition(){
        return horizontal.getCurrentPosition();
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
