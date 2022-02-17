 package teamcode.common;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.openftc.revextensions2.ExpansionHubEx;
import org.openftc.revextensions2.ExpansionHubMotor;
import org.openftc.revextensions2.RevBulkData;

import java.util.ArrayList;

import teamcode.Competition.Subsystems.ArmSystem;
import teamcode.Competition.Subsystems.EndgameSystems;
import teamcode.test.MasonTesting.CvDetectionPipeline;

public class MecanumDriveTrain {
    private static final double ANGULAR_TOLERANCE = 0.05;
    final double COUNTS_PER_INCH = 920.111004;

    /*
    This has most of the relevant information regarding a 4 wheel Mecanum DriveTrain,
    which is the most used DriveTrain in FTC
     */

    private DcMotorEx fl, fr, bl, br;
    private BNO055IMU imu;
    Localizer localizer;
    EndgameSystems systems;
    Vector2D previousVelocity;
    Vector2D previousError;
    double previousOmegaError;
    private NormalizedColorSensor sensor;
    ArmSystem arm;

    NormalizedColorSensor frontRed, backRed, frontBlue, backBlue;


    LynxModule hub;


    private boolean environmentalTerminate, eStop;
    private boolean isRed;


    /**
     * PID Constants
     *
     */
    final double pVelocity = 0.000725; //0.000725
    final double dVelocity  = 0.0; //0.027
    final double FEEDFORWARD_PID = 0.2;

    //todo for optimizing is to tune the PID aggresively due to high accel
    //todo is necessary to retune due to the rework of voltage to velocity

    public MecanumDriveTrain(HardwareMap hardwareMap){
        fl = (ExpansionHubMotor) hardwareMap.dcMotor.get("FrontLeftDrive");
        fr = (ExpansionHubMotor) hardwareMap.dcMotor.get("FrontRightDrive");
        bl = (ExpansionHubMotor) hardwareMap.dcMotor.get("BackLeftDrive");
        br = (ExpansionHubMotor) hardwareMap.dcMotor.get("BackRightDrive");
        correctMotors();

    }

    public MecanumDriveTrain(HardwareMap hardwareMap, Localizer localizer, boolean isRed){
        fl = (ExpansionHubMotor) hardwareMap.dcMotor.get("FrontLeftDrive");
        fr = (ExpansionHubMotor) hardwareMap.dcMotor.get("FrontRightDrive");
        bl = (ExpansionHubMotor) hardwareMap.dcMotor.get("BackLeftDrive");
        br = (ExpansionHubMotor) hardwareMap.dcMotor.get("BackRightDrive");


        this.localizer = localizer;
        previousVelocity = new Vector2D(0,0);
        previousOmega = 0;
        correctMotors();
        this.isRed = isRed;

    }

    /**
     * drive encoder constructor
     */
    public MecanumDriveTrain(HardwareMap hardwareMap, boolean isRed, EndgameSystems systems, ArmSystem arm){
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.RADIANS;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        this.isRed = isRed;
        this.systems = systems;

        fl = hardwareMap.get(DcMotorEx.class, "FrontLeftDrive");
        fr = hardwareMap.get(DcMotorEx.class, "FrontRightDrive");
        bl = hardwareMap.get(DcMotorEx.class, "BackLeftDrive");
        br = hardwareMap.get(DcMotorEx.class, "BackRightDrive");
        sensor = hardwareMap.get(NormalizedColorSensor.class, "color");
        sensor.setGain(1000); //325 is tested value but i think I trust this one more //280
        this.arm = arm;

        frontRed = hardwareMap.get(NormalizedColorSensor.class, "FrontColorSensorRed");
        backRed = hardwareMap.get(NormalizedColorSensor.class, "BackColorSensorRed");
        frontBlue = hardwareMap.get(NormalizedColorSensor.class, "FrontColorSensorBlue");
        backBlue = hardwareMap.get(NormalizedColorSensor.class, "BackColorSensorBlue");

        frontRed.setGain(100);
        backRed.setGain(420);
        //todo blue




        hub = hardwareMap.get(LynxModule.class, "Control Hub");
        hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);


        previousVelocity = new Vector2D(0,0);
        previousOmega = 0;
        correctMotors();
        setPIDFCoefficients(new PIDFCoefficients(2.5, 0.5, 1.0, 0));


    }

    public synchronized void rotateDistanceDERadian(double radians, double omega){
        double deltaRadians = radians - imu.getAngularOrientation().firstAngle;
        double startAngle = imu.getAngularOrientation().firstAngle;
        omega *= -getSign(deltaRadians);
        setEncoderMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setEncoderMode(DcMotor.RunMode.RUN_USING_ENCODER);
        while(Math.abs(startAngle - imu.getAngularOrientation().firstAngle) < Math.abs(deltaRadians)){
            setMotorVelocity(omega, -omega, omega, -omega);
        }
        brake();
    }

    public synchronized void rotateDistanceDE(double degrees, double omega){
        double radians = Math.toRadians(degrees);
        rotateDistanceDERadian(radians, omega);
    }


    public void spinDuck(boolean blue){
        systems.setCarouselMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        systems.setCarouselMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        int pose = -25000;
        double direction;
        DcMotor carouselEncoder;
        if(blue){
            carouselEncoder = systems.getBlueCarouselEncoder();
            direction = -1;
        }else {
            direction = 1;
            carouselEncoder = systems.getRedCarouselEncoder();
        }
        pose *= direction;

        carouselEncoder.setTargetPosition(pose);
        carouselEncoder.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while(Math.abs(carouselEncoder.getCurrentPosition()) < Math.abs(carouselEncoder.getTargetPosition()) && AbstractOpMode.currentOpMode().opModeIsActive()){
            if(Math.abs(carouselEncoder.getCurrentPosition()) < 100) {
            setStrafe(0.02);
            }else{
                brake();
            }

            if(Math.abs(carouselEncoder.getCurrentPosition()) < 10000){
                systems.runCarousel(0.1 * direction);
            }else{
                systems.runCarousel(0.5 * direction);
                systems.runCarousel(0.5 * direction);

            }
            AbstractOpMode.currentOpMode().telemetry.addData("curr", carouselEncoder.getCurrentPosition());
            AbstractOpMode.currentOpMode().telemetry.addData("tar", carouselEncoder.getTargetPosition());
            AbstractOpMode.currentOpMode().telemetry.update();

        }
        systems.runCarousel(0);
        carouselEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }


    public double getAngle(){
        return imu.getAngularOrientation().firstAngle;
    }

    /**
     * makes the drive move in an omnidirectional vector of the users choice
     * @param distance tics magnitude of the vector
     * @param degrees degrees that the vector should be rotated relative to the robots definiton of front, (rotation of 0 makes the robot drive all 4 wheels straight intake side facing)
     * @param power voltage, always should be positive
     * @param omega voltage, always should be positive
     */
    public synchronized void moveDistanceDE(int distance, double degrees, double power, double omega){
        double radians = Math.toRadians(degrees);
        power *= getSign(distance);
        Vector2D vec = Vector2D.fromAngleMagnitude(radians, power);
        double globalHeading = imu.getAngularOrientation().firstAngle;
        radians = radians + (Math.PI / 4.0) ; //45deg + globalHeading
        int flDistance = (int)(Math.sin(radians) * distance);
        int frDistance = (int)(Math.cos(radians) * distance);
        int blDistance = (int)(Math.cos(radians) * distance);
        int brDistance = (int)(Math.sin(radians) * distance);
        setEncoderMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setEncoderMode(DcMotor.RunMode.RUN_USING_ENCODER);





        LynxModule.BulkData data = hub.getBulkData();
        //AbstractOpMode.currentOpMode().telemetry.addData("fl", data.getMotorCurrentPosition(0));
        AbstractOpMode.currentOpMode().telemetry.addData("fl", flDistance);
        //AbstractOpMode.currentOpMode().telemetry.addData("fr", data.getMotorCurrentPosition(1));
        AbstractOpMode.currentOpMode().telemetry.addData("fr", frDistance);
        //AbstractOpMode.currentOpMode().telemetry.addData("bl", data.getMotorCurrentPosition(2));
        AbstractOpMode.currentOpMode().telemetry.addData("bl", blDistance);
        //AbstractOpMode.currentOpMode().telemetry.addData("br", data.getMotorCurrentPosition(3));
        AbstractOpMode.currentOpMode().telemetry.addData("br", brDistance);
        AbstractOpMode.currentOpMode().telemetry.update();

        while((Math.abs(data.getMotorCurrentPosition(0)) < Math.abs(flDistance) && Math.abs(data.getMotorCurrentPosition(1)) < Math.abs(frDistance)
        && Math.abs(data.getMotorCurrentPosition(2)) < Math.abs(blDistance) && Math.abs(data.getMotorCurrentPosition(3)) < Math.abs(brDistance))){
            hub.clearBulkCache();
            data = hub.getBulkData();
            AbstractOpMode.currentOpMode().telemetry.addData("fl",- data.getMotorCurrentPosition(0));
            //AbstractOpMode.currentOpMode().telemetry.addData("fl", flDistance);
            AbstractOpMode.currentOpMode().telemetry.addData("fr", data.getMotorCurrentPosition(1));
            //AbstractOpMode.currentOpMode().telemetry.addData("fr", frDistance);
            AbstractOpMode.currentOpMode().telemetry.addData("bl", -data.getMotorCurrentPosition(2));
            //AbstractOpMode.currentOpMode().telemetry.addData("bl", blDistance);
            AbstractOpMode.currentOpMode().telemetry.addData("br", data.getMotorCurrentPosition(3));
            //AbstractOpMode.currentOpMode().telemetry.addData("br", brDistance);
            AbstractOpMode.currentOpMode().telemetry.update();
            setPower(vec, 0);
        }

        brake();
        rotateDistanceDERadian(globalHeading, omega);
    }

    private void setPIDFCoefficients(PIDFCoefficients coefficients){
        fl.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, coefficients);
        fr.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, coefficients);
        bl.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, coefficients);
        br.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, coefficients);
    }

    public synchronized void moveDistanceDEVelocity(int distance, double degrees, double velocity){
        double radians = Math.toRadians(degrees);
        velocity *= getSign(distance);
        Vector2D vec = Vector2D.fromAngleMagnitude(radians, velocity);
        radians = radians + (Math.PI / 4.0) ; //45deg + globalHeading
        int flDistance = (int)(Math.sin(radians) * distance);
        int frDistance = (int)(Math.cos(radians) * distance);
        int blDistance = (int)(Math.cos(radians) * distance);
        int brDistance = (int)(Math.sin(radians) * distance);
        setEncoderMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setEncoderMode(DcMotor.RunMode.RUN_USING_ENCODER);
        hub.clearBulkCache();
        LynxModule.BulkData data = hub.getBulkData();
        while(Math.abs(data.getMotorCurrentPosition(0))< Math.abs(flDistance) || Math.abs(data.getMotorCurrentPosition(1)) < Math.abs(frDistance) ||
                Math.abs(data.getMotorCurrentPosition(2)) < Math.abs(blDistance) || Math.abs(data.getMotorCurrentPosition(3)) < Math.abs(brDistance)){
            hub.clearBulkCache();
            data = hub.getBulkData();
//            AbstractOpMode.currentOpMode().telemetry.addData("fl",data.getMotorCurrentPosition(0));
//            //AbstractOpMode.currentOpMode().telemetry.addData("fl", flDistance);
//            AbstractOpMode.currentOpMode().telemetry.addData("fr", data.getMotorCurrentPosition(1));
//            //AbstractOpMode.currentOpMode().telemetry.addData("fr", frDistance);
//            AbstractOpMode.currentOpMode().telemetry.addData("bl", data.getMotorCurrentPosition(2));
//            //AbstractOpMode.currentOpMode().telemetry.addData("bl", blDistance);
//            AbstractOpMode.currentOpMode().telemetry.addData("br", data.getMotorCurrentPosition(3));
//            //AbstractOpMode.currentOpMode().telemetry.addData("br", brDistance);
//            AbstractOpMode.currentOpMode().telemetry.update();

            setVelocity(vec, 0);
        }
        brake();
    }
    public synchronized void driveColorSensor(double pow){
        setEncoderMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setEncoderMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        arm.lowerLinkage();
        arm.intakeDumb(1.0);
        boolean detectedElement = false;
        setMotorVelocity(pow,pow,pow,pow);
        Utils.sleep(250);
        while(!detectedElement){
//            Vector2D vec = Vector2D.fromAngleMagnitude(0, pow);
//            setEncoderMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//            setEncoderMode(DcMotor.RunMode.RUN_USING_ENCODER);
//            hub.clearBulkCache();
//            LynxModule.BulkData data = hub.getBulkData();
//            while(Math.abs(data.getMotorCurrentPosition(0))< Math.abs(200) || Math.abs(data.getMotorCurrentPosition(1)) < Math.abs(200) ||
//                    Math.abs(data.getMotorCurrentPosition(2)) < Math.abs(200) || Math.abs(data.getMotorCurrentPosition(3)) < Math.abs(200)){
//                hub.clearBulkCache();
//                data = hub.getBulkData();
                setMotorVelocity(pow,pow,pow,pow);
                NormalizedRGBA colors = sensor.getNormalizedColors();
                double red = colors.red;
                double blue = colors.blue;
                double green = colors.green;
                AbstractOpMode.currentOpMode().telemetry.addData("green", green);
                AbstractOpMode.currentOpMode().telemetry.addData("red", red);
                AbstractOpMode.currentOpMode().telemetry.addData("blue", blue);
                AbstractOpMode.currentOpMode().telemetry.update();

                if (red > 0.9) {
                    detectedElement = true;
                    break;
                } else {
                    detectedElement = false;
                }
                //setVelocity(vec, 0);
            //}
        }
        brake();
        AbstractOpMode.currentOpMode().telemetry.clear();
        Debug.log("grabbed");
        hub.clearBulkCache();
        LynxModule.BulkData data = hub.getBulkData();
        double posAvg = 0;
        for(int i = 0; i < 4; i++){
            int pos = Math.abs(data.getMotorCurrentPosition(i));
            Debug.log(pos);
            posAvg += pos;
        }
        posAvg = posAvg / (4.0 * 0.707); //4 cos 45
        arm.preScoreAuto();
        arm.intakeDumb(-1.0);
//        posAvg -= 600;
//        if(posAvg < 600){
//            posAvg = 0;
//        }

        ArrayList<Movement> spline = new ArrayList<>();

        moveDistanceDEVelocity(200, 180, 9);
        strafeDistanceSensor(6, 0);
        moveDistanceDEVelocity((int)((posAvg) / Math.cos(Math.toRadians(10))), 170, 9);
//        spline.add(new Movement(200, 3,180));
//        spline.add(new Movement((int)(posAvg / Math.cos(10)), 3, 170));
//        splicedMovement(spline);
        Debug.log("val" + (int)((posAvg) / Math.cos(Math.toRadians(10))));

        arm.intakeDumb(0);
    }

    public void semiDumbVisionDriving(double pow){
        while(!somethingInThePartition()){ //add the  conditional with something in the partition
            setStrafe(pow);
        }
        setEncoderMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setEncoderMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        boolean detectedElement = false;
        while(!detectedElement){
            arm.lowerLinkage();
            arm.intakeDumb(1.0);
            NormalizedRGBA colors = sensor.getNormalizedColors();
            double green = colors.green;
            if (green > 0.9) {
                detectedElement = true;
            } else {
                detectedElement = false;
            }
            setPower(pow, pow, pow, pow);
        }
        hub.clearBulkCache();
        LynxModule.BulkData data = hub.getBulkData();
        int posAvg = 0;
        for(int i = 0; i < 4; i++){
            int pos = data.getMotorCurrentPosition(i);
            posAvg += pos;
        }
        posAvg = posAvg / 4;
        moveDistanceDEVelocity(-posAvg, 0, 6);
        strafeDistanceSensor(0.3, 0);
    }
    private boolean somethingInThePartition(){
        //replace this with something that actually works Mason
        return true;
    }
    public void SmartVisionDriving(double pow, Object pipeline){
        double theta = 0;
        pipeline.getClass(); //getThetaValue()
        int tics = inchesToTics();
        theta += Math.toRadians(5);
        moveDistanceDE(tics, theta, pow, 0);
        setEncoderMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setEncoderMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        boolean detectedElement = false;
        while(!detectedElement){
            arm.lowerLinkage();
            arm.intakeDumb(1.0);
            NormalizedRGBA colors = sensor.getNormalizedColors();
            double green = colors.green;
            if (green > 0.9) {
                detectedElement = true;
            } else {
                detectedElement = false;
            }
            setPower(pow, pow, pow, pow);
        }
        hub.clearBulkCache();
        LynxModule.BulkData data = hub.getBulkData();
        int posAvg = 0;
        for(int i = 0; i < 4; i++){
            int pos = data.getMotorCurrentPosition(i);
            posAvg += pos;
        }
        posAvg = posAvg / 4;
        double secondDistance = tics * Math.cos(theta);
        moveDistanceDEVelocity(-(int)(posAvg + secondDistance), 0, 6);
        strafeDistanceSensor(0.3, 0);
    }
    private int inchesToTics(){
        return 0;
        //TODO import this from localizer
    }




    public synchronized void moveDistanceDENoErrorCorrection(int distance, double degrees, double power){
        double radians = Math.toRadians(degrees);
        power *= getSign(distance);
        Vector2D vec = Vector2D.fromAngleMagnitude(radians, power);
        double globalHeading = imu.getAngularOrientation().firstAngle;
        radians = radians + (Math.PI / 4.0) ; //45deg + globalHeading
        int flDistance = (int)(Math.sin(radians) * distance);
        int frDistance = (int)(Math.cos(radians) * distance);
        int blDistance = (int)(Math.cos(radians) * distance);
        int brDistance = (int)(Math.sin(radians) * distance);
        setEncoderMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setEncoderMode(DcMotor.RunMode.RUN_USING_ENCODER);



        LynxModule.BulkData data = hub.getBulkData();
        //AbstractOpMode.currentOpMode().telemetry.addData("fl", data.getMotorCurrentPosition(0));
        AbstractOpMode.currentOpMode().telemetry.addData("fl", flDistance);
        //AbstractOpMode.currentOpMode().telemetry.addData("fr", data.getMotorCurrentPosition(1));
        AbstractOpMode.currentOpMode().telemetry.addData("fr", frDistance);
        //AbstractOpMode.currentOpMode().telemetry.addData("bl", data.getMotorCurrentPosition(2));
        AbstractOpMode.currentOpMode().telemetry.addData("bl", blDistance);
        //AbstractOpMode.currentOpMode().telemetry.addData("br", data.getMotorCurrentPosition(3));
        AbstractOpMode.currentOpMode().telemetry.addData("br", brDistance);
        AbstractOpMode.currentOpMode().telemetry.update();

        while((Math.abs(data.getMotorCurrentPosition(0)) < Math.abs(flDistance) && Math.abs(data.getMotorCurrentPosition(1)) < Math.abs(frDistance)
                && Math.abs(data.getMotorCurrentPosition(2)) < Math.abs(blDistance) && Math.abs(data.getMotorCurrentPosition(3)) < Math.abs(brDistance))){
            hub.clearBulkCache();
            data = hub.getBulkData();
            AbstractOpMode.currentOpMode().telemetry.addData("fl",- data.getMotorCurrentPosition(0));
            //AbstractOpMode.currentOpMode().telemetry.addData("fl", flDistance);
            AbstractOpMode.currentOpMode().telemetry.addData("fr", data.getMotorCurrentPosition(1));
            //AbstractOpMode.currentOpMode().telemetry.addData("fr", frDistance);
            AbstractOpMode.currentOpMode().telemetry.addData("bl", -data.getMotorCurrentPosition(2));
            //AbstractOpMode.currentOpMode().telemetry.addData("bl", blDistance);
            AbstractOpMode.currentOpMode().telemetry.addData("br", data.getMotorCurrentPosition(3));
            //AbstractOpMode.currentOpMode().telemetry.addData("br", brDistance);
            AbstractOpMode.currentOpMode().telemetry.update();


            setPower(vec, 0);
        }

        brake();
    }

    public void cleanup(){

        imu.close();
        hub.clearBulkCache();

    }

    public void setEncoderMode(DcMotor.RunMode runMode){
        fl.setMode(runMode);
        fr.setMode(runMode);
        bl.setMode(runMode);
        br.setMode(runMode);
    }



    public MecanumDriveTrain(HardwareMap hardwareMap, Localizer localizer, boolean isRed, EndgameSystems systems){
        this(hardwareMap, localizer, isRed);
        this.systems = systems;
    }

    CvDetectionPipeline pipeline;

    public MecanumDriveTrain(HardwareMap hardwareMap, Localizer localizer, CvDetectionPipeline pipeline, boolean isRed){
        //this(hardwareMap, localizer, isRed);
        this.pipeline = pipeline;
    }
/*
        fl = (ExpansionHubMotor) hardwareMap.dcMotor.get("FrontLeftDrive");
        fr = (ExpansionHubMotor) hardwareMap.dcMotor.get("FrontRightDrive");
        bl = (ExpansionHubMotor) hardwareMap.dcMotor.get("BackLeftDrive");
        br = (ExpansionHubMotor) hardwareMap.dcMotor.get("BackRightDrive");

        this.localizer = localizer;
        previousVelocity = new Vector2D(0,0);
        previousOmega = 0;
        correctMotors();

 */

    public synchronized void smartDuck(boolean blue){
        systems.setCarouselMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        systems.setCarouselMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        double direction;
        if(blue){
            direction = -1;
        }else{
            direction = 1;
        }
        systems.runCarousel(-0.1);
        Utils.sleep(100);
        double currentTicks = systems.getCarouselPos();
        double previousTicks = 0;
        while(Math.abs(currentTicks - previousTicks) > 50){
            currentTicks = systems.getCarouselPos();
            setStrafe(0.2 * direction);
            previousTicks = currentTicks;
            AbstractOpMode.currentOpMode().telemetry.addData("dc", currentTicks - previousTicks);
            AbstractOpMode.currentOpMode().telemetry.addData("why", systems.getCarouselPos());
            AbstractOpMode.currentOpMode().telemetry.update();
        }
        Utils.sleep(250);
        setStrafe(0);
        while(Math.abs(currentTicks - previousTicks) < 50){
            previousTicks = currentTicks;
            currentTicks = systems.getCarouselPos();
            setStrafe(-0.2 * direction);
            AbstractOpMode.currentOpMode().telemetry.addData("dc", Math.abs(currentTicks - previousTicks));
            AbstractOpMode.currentOpMode().telemetry.addData("why", systems.getCarouselPos());
            AbstractOpMode.currentOpMode().telemetry.update();
        }
        setStrafe(0);
        AbstractOpMode.currentOpMode().telemetry.clear();

        systems.runCarousel(0);
        systems.scoreDuckAuto();
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
        environmentalTerminate = false;
        while(pipeline.yPointList().get(0) < 5 && AbstractOpMode.currentOpMode().opModeIsActive() && !eStop && !environmentalTerminate) { //todo ask mason how to ensure I am tracking the same cube every time here
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
        environmentalTerminate = false;
        while(xPartitionDeviation > 5 && AbstractOpMode.currentOpMode().opModeIsActive() && !eStop && !environmentalTerminate){
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


    public synchronized void strafeDistanceSensor(double omega, double radians){


        environmentalTerminate = false;
        double distanceFrontThreshold = 0.95;
        double distanceBackThreshold = 0.95;
        double lowMagnitudeFrontReading = 0;
        double lowMagnitudeBackReading = 0;
        //todo calibrate the tolerance of it.
        AbstractOpMode.currentOpMode().telemetry.clear();
        setEncoderMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setEncoderMode(DcMotor.RunMode.RUN_USING_ENCODER);

        NormalizedRGBA frontRGBA;
        NormalizedRGBA backRGBA;
            if(isRed) {
                frontRGBA = frontRed.getNormalizedColors();
                backRGBA = backRed.getNormalizedColors();
            }else{
                frontRGBA = frontBlue.getNormalizedColors();
                backRGBA = backBlue.getNormalizedColors();
            }
            while (frontRGBA.green < distanceFrontThreshold && backRGBA.green < distanceBackThreshold){

                if(isRed) {
                    frontRGBA = frontRed.getNormalizedColors();
                    backRGBA = backRed.getNormalizedColors();
                }else{
                    frontRGBA = frontBlue.getNormalizedColors();
                    backRGBA = backBlue.getNormalizedColors();
                }
                AbstractOpMode.currentOpMode().telemetry.addData("FGreen", frontRGBA.green);
                AbstractOpMode.currentOpMode().telemetry.addData("BGreen", backRGBA.green);
                AbstractOpMode.currentOpMode().telemetry.update();

                Vector2D vec = Vector2D.fromAngleMagnitude(radians + (Math.PI / 2.0), omega);
                setVelocity(vec, 0);
                Utils.sleep(100);

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
        fl.setDirection(DcMotorSimple.Direction.REVERSE);
        fr.setDirection(DcMotorSimple.Direction.REVERSE);
        bl.setDirection(DcMotorSimple.Direction.REVERSE);
        br.setDirection(DcMotorSimple.Direction.REVERSE);

    }






    /**
     * moving from position to position, assuming some kind of translational motion
     * NEW no rotation involved since we can handle that as a single dimensional operation seperately rather than integrating it into the 3 dimensional operation it was
     * @param desiredPosition the end point of the robot
     * @param desiredVelocity the end velocity of the robot in inches per second
     *
     */

    boolean maxReached;
    Vector2D previousPosition;
    int lowMagnitudeHardwareCycles;

    public synchronized void moveToPosition(Vector2D desiredPosition, double desiredVelocity){

        if(localizer == null){
            return;
        }
        RobotPositionStateUpdater.RobotPositionState currentState = localizer.getCurrentState();
        Vector2D desiredPositionPointer = new Vector2D(desiredPosition.getX() - currentState.getPosition().getX() , desiredPosition.getY() - currentState.getPosition().getY());
        Vector2D newDesiredPosition = desiredPosition.add(new Vector2D(5.0 * Math.cos(desiredPositionPointer.getDirection()), 5.0 * Math.sin(desiredPositionPointer.getDirection())));
        maxReached = false;
        previousError = new Vector2D(0,0);
        Vector2D steadyStateError = new Vector2D(0,0);
        previousOmegaError = 0;
        environmentalTerminate = false;
        double heading = currentState.getRotation();
        Debug.log(desiredVelocity);
        lowMagnitudeHardwareCycles = 0;
        previousPosition = currentState.getPosition();

        while((Math.abs(newDesiredPosition.subtract(currentState.getPosition()).magnitude()) > 5.0) && AbstractOpMode.currentOpMode().opModeIsActive() && !eStop && !environmentalTerminate){

            currentState = localizer.getCurrentState();
            Vector2D position = currentState.getPosition();
            Vector2D deltaPosition = position.subtract(previousPosition);
            Vector2D positionError = desiredPosition.subtract(currentState.getPosition());
            double errorAngle = positionError.getDirection() - heading;
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

//            if(maxReached){
//                error = new Vector2D(0,0);
//            }



            //found and fixed stupid math error
            Vector2D passedVector = previousVelocity.add(new Vector2D(error.getX(), error.getY()));

            if(passedVector.magnitude() > 1.0){
                passedVector = passedVector.normalize();
                desiredVelocity = recordedVelocity.magnitude();
            }
            previousVelocity = setPower(passedVector,0.0);

           // previousVelocity.multiply(sign);
            previousError = error;
            previousPosition = new Vector2D(position.getX(), position.getY());


//            AbstractOpMode.currentOpMode().telemetry.addData("", currentState.toString());
//            AbstractOpMode.currentOpMode().telemetry.addData("dpos", deltaPosition.magnitude());
//            AbstractOpMode.currentOpMode().telemetry.addData("", lowMagnitudeHardwareCycles);


            //AbstractOpMode.currentOpMode().telemetry.addData("distance", Math.abs(newDesiredPosition.subtract(currentState.getPosition()).magnitude()));
            //AbstractOpMode.currentOpMode().telemetry.addData("sign", Math.abs(newDesiredPosition.subtract(currentState.getPosition()).magnitude()));
//            AbstractOpMode.currentOpMode().telemetry.addData("", currentState.toString());
//            AbstractOpMode.currentOpMode().telemetry.addData("", error);
//            AbstractOpMode.currentOpMode().telemetry.addData("", Math.abs(newDesiredPosition.subtract(currentState.getPosition()).magnitude()));




            AbstractOpMode.currentOpMode().telemetry.update();


        }
        //AbstractOpMode.currentOpMode().telemetry.clear();
        //Debug.log("done");
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

        fl.setVelocity(-flVelocity * WHEEL_RADIUS_IN, AngleUnit.RADIANS);
        fr.setVelocity(frVelocity * WHEEL_RADIUS_IN, AngleUnit.RADIANS);
        bl.setVelocity(-blVelocity * WHEEL_RADIUS_IN, AngleUnit.RADIANS);
        br.setVelocity(brVelocity * WHEEL_RADIUS_IN, AngleUnit.RADIANS);
    }


    double previousOmega;
    double pRotation;
    public synchronized void moveToRotation(double desiredRotation, double omega){
        if(localizer == null){
            return;
        }
        RobotPositionStateUpdater.RobotPositionState state = localizer.getCurrentState();
        previousOmega = 0;
        environmentalTerminate = false;
        while(Math.abs(desiredRotation - state.getRotation()) > 0.05 && AbstractOpMode.currentOpMode().opModeIsActive() && !eStop && !environmentalTerminate){
            state = localizer.getCurrentState();
            double recordedOmega = state.getAngularVelocity();
            double omegaError = omega - recordedOmega;
            omegaError *= pRotation;
            omega += omegaError;
            setVelocity(new Vector2D(0,0), omega);
//            AbstractOpMode.currentOpMode().telemetry.addData("", state.toString());
//            AbstractOpMode.currentOpMode().telemetry.update();
        }
        brake();
    }

    public synchronized void rotateDistance(double radians, double power){
        RobotPositionStateUpdater.RobotPositionState state = localizer.getCurrentState();

        environmentalTerminate = false;

        while(Math.abs((state.getRotation() - radians))  > 0.05 && AbstractOpMode.currentOpMode().opModeIsActive() && !environmentalTerminate && !eStop){
            state = localizer.getCurrentState();
//            AbstractOpMode.currentOpMode().telemetry.addData("", state);
//            AbstractOpMode.currentOpMode().telemetry.update();
            setPower(power, -power, power, -power);
        }
        brake();


    }

    public void brake() {
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
    public Vector2D setPower(Vector2D velocity, double turnValue){
        turnValue = -turnValue;
        double direction = velocity.getDirection();



        double power = velocity.magnitude();

        double angle = direction + ( Math.PI / 4.0);
        double sin = Math.sin(angle);
        double cos = Math.cos(angle);

        setPower((power * sin - turnValue),(power * cos + turnValue),
                (power * cos - turnValue), (power * sin + turnValue));
        return new Vector2D(velocity.getX(), velocity.getY());
    }

    public Vector2D setPower(Vector2D velocity, double turnValue, double robotHeading){
        turnValue = -turnValue;
        double direction = velocity.getDirection() + robotHeading;



        double power = velocity.magnitude();

        double angle = direction + ( Math.PI / 4.0);
        double sin = Math.sin(angle);
        double cos = Math.cos(angle);

        setPower((power * sin - turnValue),(power * cos + turnValue),
                (power * cos - turnValue), (power * sin + turnValue));
        return new Vector2D(velocity.getX(), velocity.getY());
    }


    public void setPower(double flPow, double frPow, double blPow, double brPow) {
        fl.setPower(-flPow);
        fr.setPower(frPow);
        bl.setPower(-blPow);
        br.setPower(brPow);
    }

    public double setStrafe(double val){
        if(!isRed){
            setPower(-val, val, val, -val);
        }else {
            setPower(val, -val, -val, val);
        }
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

    private static final double WHEELBASE_X = 10.5;
    private static final double WHEELBASE_Y = 13.25;

    /**
     * arcs in auto
     * @param motion the angle of the chord and the magnitude of the velocity
     * @param omega rotational velocity to be applied, should be at a 1:1 with velocity magnitude for
     *              circular arcs
     * @param arclength median arclength between the wheelbases
     * @param dAngle angle of the arc in degrees
     */

    public void arcDriving(Vector2D motion, double omega, int arclength, double dAngle){
        dAngle = Math.toRadians(dAngle);
        double[][] inverseKinematicModelArr = new double[][]{
                {1,-1, -(WHEELBASE_X + WHEELBASE_Y)},
                {1,1, (WHEELBASE_X + WHEELBASE_Y)},
                {1,1, -(WHEELBASE_X + WHEELBASE_Y)},
                {1,-1, (WHEELBASE_X + WHEELBASE_Y)}};
        double[][] motionModelArr = new double[][]{{motion.getX()}, {motion.getY()}, {omega}};

        Matrix inverseKinematicModel = new Matrix(inverseKinematicModelArr);
        Matrix motionModel = new Matrix(motionModelArr);

        Matrix wheelMotions = inverseKinematicModel.multiply(motionModel);

        wheelMotions.multiply(1.0 / WHEEL_RADIUS_IN);

        setEncoderMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setEncoderMode(DcMotor.RunMode.RUN_USING_ENCODER);

        double minArcLength = arclength - (WHEELBASE_X / 2.0) * dAngle;
        double maxArcLength = arclength + (WHEELBASE_X / 2.0) * dAngle;

        hub.clearBulkCache();
        LynxModule.BulkData data = hub.getBulkData();

        while(data.getMotorCurrentPosition(0) < minArcLength || data.getMotorCurrentPosition(1) < maxArcLength ||
                data.getMotorCurrentPosition(2) < minArcLength || data.getMotorCurrentPosition(3) < maxArcLength) {
            hub.clearBulkCache();
            data = hub.getBulkData();
            setMotorVelocity(wheelMotions.getValue(0, 0), wheelMotions.getValue(1, 0),
                    wheelMotions.getValue(2, 0), wheelMotions.getValue(3, 0));
        }
        brake();
    }



    public synchronized void splicedMovement(ArrayList<Movement> movements){
        hub.clearBulkCache();
        LynxModule.BulkData data = hub.getBulkData();
        setEncoderMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setEncoderMode(DcMotor.RunMode.RUN_USING_ENCODER);
        for(int i = 0; i < movements.size(); i++){
            Movement curr = movements.get(i);
            if(curr.getMovement() == Movement.MovementType.TRANSLATION) {
                double distance = curr.getDistance();
                double radians = curr.getRadians();
                int flDistance = (int) (Math.sin(radians) * distance);
                int frDistance = (int) (Math.cos(radians) * distance);
                int blDistance = (int) (Math.cos(radians) * distance);
                int brDistance = (int) (Math.sin(radians) * distance);

                hub.clearBulkCache();
                data = hub.getBulkData();
                int flInit = data.getMotorCurrentPosition(0);
                int frInit = data.getMotorCurrentPosition(1);
                int blInit = data.getMotorCurrentPosition(2);
                int brInit = data.getMotorCurrentPosition(3);
                Vector2D vec = Vector2D.fromAngleMagnitude(radians, curr.getVelocity());

                while (Math.abs(data.getMotorCurrentPosition(0) - flInit) < flDistance ||
                        Math.abs(data.getMotorCurrentPosition(1) - frInit) < frDistance ||
                        Math.abs(data.getMotorCurrentPosition(2) - blInit) < blDistance ||
                        Math.abs(data.getMotorCurrentPosition(3) - brInit) < brDistance){

                    hub.clearBulkCache();
                    data = hub.getBulkData();

                    setVelocity(vec, 0);
                }
            }else if(curr.getMovement() == Movement.MovementType.ROTATION){
                double omega = curr.getOmega();
                double deltaRadians = curr.getRotation() - imu.getAngularOrientation().firstAngle;
                double startAngle = imu.getAngularOrientation().firstAngle;
                omega *= -getSign(deltaRadians);
                while(Math.abs(startAngle - imu.getAngularOrientation().firstAngle) < Math.abs(deltaRadians)){
                    setMotorVelocity(omega, -omega, omega, -omega);
                }
            }else{
                Utils.sleep(curr.getMillis());
            }
        }
        brake();
    }

    public void setEnvironmentalTerminate(boolean val){
        environmentalTerminate = val;
    }

    public void seteStop(boolean val){
        eStop = val;
    }
}
