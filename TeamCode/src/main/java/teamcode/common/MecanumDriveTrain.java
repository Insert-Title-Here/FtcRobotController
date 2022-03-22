 package teamcode.common;

import android.graphics.Color;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
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

import static java.lang.Math.PI;

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
    private NormalizedColorSensor sensor, warehouse;
    ArmSystem arm;

    ColorRangeSensor frontRed, backRed, frontBlue, backBlue;


    LynxModule hub;


    private boolean environmentalTerminate, eStop;
    private boolean isRed;
    private volatile boolean[] flags;


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
     * drive encoder constructor with Modular PIDF controller constants, for seperate opModes
     */
    public MecanumDriveTrain(HardwareMap hardwareMap, boolean isRed, EndgameSystems systems, ArmSystem arm, PIDFCoefficients coefficients){
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.RADIANS;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "AdafruitIMUCalibration.json"; // see the calibration sample opmode
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
        this.arm = arm;

        frontRed = hardwareMap.get(ColorRangeSensor.class, "FrontColorSensorRed");
        backRed = hardwareMap.get(ColorRangeSensor.class, "BackColorSensorRed");
        frontBlue = hardwareMap.get(ColorRangeSensor.class, "FrontColorSensorBlue");
        backBlue = hardwareMap.get(ColorRangeSensor.class, "BackColorSensorBlue");
        warehouse = hardwareMap.get(NormalizedColorSensor.class, "WarehouseTapeSensor");
        sensor = hardwareMap.get(NormalizedColorSensor.class, "color");

        warehouse.setGain(600);
        frontRed.setGain(200);
        backRed.setGain(520);
        frontBlue.setGain(100);
        backBlue.setGain(300);
        sensor.setGain(950); //325 is tested value but i think I trust this one more //280

        flags = new boolean[]{false, false, false, false, false};


        hub = hardwareMap.get(LynxModule.class, "Control Hub");
        hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);


        previousVelocity = new Vector2D(0,0);
        previousOmega = 0;
        correctMotors();
        setPIDFCoefficients(coefficients);
        Debug.log("here");

    }

    public MecanumDriveTrain(HardwareMap hardwareMap, boolean isRed, EndgameSystems systems, ArmSystem arm){
        this(hardwareMap, isRed, systems, arm, new PIDFCoefficients(2.5, 0.5, 1.0, 0));

    }


    public synchronized void rotateDistanceDERadian(double radians, double omega){
        double deltaRadians = radians - imu.getAngularOrientation().firstAngle;
        double startAngle = imu.getAngularOrientation().firstAngle;
        omega *= -getSign(deltaRadians);
        setEncoderMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setEncoderMode(DcMotor.RunMode.RUN_USING_ENCODER);

        while(Math.abs(startAngle - imu.getAngularOrientation().firstAngle) < Math.abs(deltaRadians) && opModeIsRunning()){
            double angularDistance = Math.abs(startAngle - imu.getAngularOrientation().firstAngle);
            double radialDistance = Math.abs(deltaRadians);
            double ratio = (radialDistance - angularDistance) / angularDistance;
            if(ratio < 0.2){
                ratio = 0.2;
            }else if(ratio > 1.0){
                ratio = 1.0;
            }
            AbstractOpMode.currentOpMode().telemetry.addData("ratio", ratio);
            AbstractOpMode.currentOpMode().telemetry.addData("angular", angularDistance);
            AbstractOpMode.currentOpMode().telemetry.update();
            setMotorVelocity(omega * ratio, -omega * ratio, omega * ratio, -omega * ratio);


        }
        brake();
    }

    public synchronized void rotateDistanceDE(double degrees, double omega){
        double radians = Math.toRadians(degrees);
        rotateDistanceDERadian(radians, omega);
    }



    public synchronized void driveColorSensorWarehouse(double velocity){
        driveColorSensorWarehouse(velocity, true);
    }

    public synchronized void driveColorSensorWarehouse(double velocity, boolean isBrake){
        NormalizedRGBA rgba = warehouse.getNormalizedColors();
        setEncoderMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setEncoderMode(DcMotor.RunMode.RUN_USING_ENCODER);
        while(rgba.red < 0.9 && opModeIsRunning()){
            rgba = warehouse.getNormalizedColors();
            setMotorVelocity(velocity,velocity,velocity,velocity);
        }
        if(isBrake) {
            brake();
        }
    }

     public synchronized void strafeColorSensorWarehouse(double velocity){
         NormalizedRGBA rgba = warehouse.getNormalizedColors();
         setEncoderMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
         setEncoderMode(DcMotor.RunMode.RUN_USING_ENCODER);
         while(rgba.red < 0.9 && opModeIsRunning()){
             rgba = warehouse.getNormalizedColors();
             setMotorVelocity(velocity,-velocity,-velocity,velocity);
         }
         brake();
     }

    public synchronized void driveColorSensorBlue(double velocity){
        warehouse.setGain(400);
        NormalizedRGBA rgba = warehouse.getNormalizedColors();
        setEncoderMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setEncoderMode(DcMotor.RunMode.RUN_USING_ENCODER);
        while(rgba.blue < 0.9 && opModeIsRunning()){
            rgba = warehouse.getNormalizedColors();
            setMotorVelocity(velocity,velocity,velocity,velocity);
        }
        warehouse.setGain(600);
        brake();
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
        while(Math.abs(carouselEncoder.getCurrentPosition()) < Math.abs(carouselEncoder.getTargetPosition()) && opModeIsRunning()){
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
        && Math.abs(data.getMotorCurrentPosition(2)) < Math.abs(blDistance) && Math.abs(data.getMotorCurrentPosition(3)) < Math.abs(brDistance))
        && opModeIsRunning()){
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

    /**
     * movement in any translational direction reaching any desired velocity
     * @param distance is measured in rotational encoder tics,
     * this is to simplify the method implementation
     *
     * @param degrees the angle the robot should be travelling at in DEGREES,
     * this parameter should be passed [-180, 180]
     * KEY FOR THIS PARAMETER
     * 0, Drive straight
     * 90, strafe right
     * 180, drive backwards
     * -90, strafe left
     * any number in between is a diagonal in that quadrant.
     *
     * @param velocity the linear velocity in inches per second that the robot should travel
     */
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
        while((Math.abs(data.getMotorCurrentPosition(0))< Math.abs(flDistance) || Math.abs(data.getMotorCurrentPosition(1)) < Math.abs(frDistance) ||
                Math.abs(data.getMotorCurrentPosition(2)) < Math.abs(blDistance) || Math.abs(data.getMotorCurrentPosition(3)) < Math.abs(brDistance)) && opModeIsRunning()){
            hub.clearBulkCache();
            data = hub.getBulkData();
//            AbstractOpMode.currentOpMode().telemetry.addData("fl",data.getMotorCurrentPosition(0));
//            AbstractOpMode.currentOpMode().telemetry.addData("fl", flDistance);
//            AbstractOpMode.currentOpMode().telemetry.addData("fr", data.getMotorCurrentPosition(1));
//            AbstractOpMode.currentOpMode().telemetry.addData("fr", frDistance);
//            AbstractOpMode.currentOpMode().telemetry.addData("bl", data.getMotorCurrentPosition(2));
//            AbstractOpMode.currentOpMode().telemetry.addData("bl", blDistance);
//            AbstractOpMode.currentOpMode().telemetry.addData("br", data.getMotorCurrentPosition(3));
//            AbstractOpMode.currentOpMode().telemetry.addData("br", brDistance);
//            AbstractOpMode.currentOpMode().telemetry.update();

            setVelocity(vec, 0);
        }
        brake();
    }

    private boolean opModeIsRunning(){
        return AbstractOpMode.currentOpMode().opModeIsActive() && !AbstractOpMode.currentOpMode().isStopRequested();
    }


    public synchronized void driveColorSensor(double pow){
        warehouse.setGain(700);
        setEncoderMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setEncoderMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        arm.lowerLinkage();
        arm.intakeDumb(1.0);
        boolean detectedElement = false;
        Utils.sleep(100);
        setMotorVelocity(pow,pow,pow,pow);
        Utils.sleep(250);
        double amps = arm.getMilliAmps();
        boolean isAmp = false;
        while(!detectedElement && opModeIsRunning()){
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

                amps = arm.getMilliAmps();
                if(amps > 3000){
                    isAmp = true;
                }

                if (red > 0.9 && isAmp) {
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
        double sign = 1;
        if(!isRed){
            sign = -1;
        }
        moveDistanceDEVelocity(200, 180, 9);

        strafeDistanceSensor(6, 0);
        driveColorSensorWarehouse(-6);
        moveDistanceDEVelocity(300, 180,9);
        //moveDistanceDEVelocity((int)((posAvg) / Math.cos(Math.toRadians(10))), 170 * sign, 9);
//        spline.add(new Movement(200, 3,180));
//        spline.add(new Movement((int)(posAvg / Math.cos(10)), 3, 170));
//        splicedMovement(spline);
        //Debug.log("val" + (int)((posAvg) / Math.cos(Math.toRadians(10))));

        arm.intakeDumb(0);
    }

    public synchronized void driveColorSensorNoWarehouse(double pow){
        warehouse.setGain(800);
        setEncoderMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setEncoderMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        arm.lowerLinkage();
        arm.intakeDumb(1.0);
        boolean detectedElement = false;
        Utils.sleep(100);
        setMotorVelocity(pow,pow,pow,pow);
        Utils.sleep(250);
        double amps = arm.getMilliAmps();
        boolean isAmp = false;
        while(!detectedElement && opModeIsRunning()){
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
        hub.clearBulkCache();
        LynxModule.BulkData data = hub.getBulkData();
        double posAvg = 0;
        for(int i = 0; i < 4; i++){
            int pos = Math.abs(data.getMotorCurrentPosition(i));
            Debug.log(pos);
            posAvg += pos;
        }
        Debug.log("posavg" + posAvg);
        posAvg = posAvg / (4.0 * 0.707); //4 cos 45
        arm.preScore();
        arm.intakeDumb(-1.0);
//        posAvg -= 600;
//        if(posAvg < 600){
//            posAvg = 0;
//        }

        ArrayList<Movement> spline = new ArrayList<>();
        double sign = 1;
        if(!isRed){
            sign = -1;
        }
        posAvg = posAvg - 650;
        if(posAvg < 0){
            posAvg = 0;
        }

        //strafeDistanceSensorOpposite(6, 0);
//        moveDistanceDEVelocity((int)posAvg, 180, 6);
//        spline.add(new Movement(200, 3,180));
//        spline.add(new Movement((int)(posAvg / Math.cos(10)), 3, 170));
//        splicedMovement(spline);
        //Debug.log("val" + (int)((posAvg) / Math.cos(Math.toRadians(10))));
        //moveDistanceDEVelocity(200, 180, 6);
        arm.intakeDumb(0);
    }


    public synchronized void driveColorSensorSpliced(double pow){
        setEncoderMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setEncoderMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        arm.lowerLinkage();
        arm.intakeDumb(1.0);
        boolean detectedElement = false;
        Utils.sleep(100);
        setMotorVelocity(pow,pow,pow,pow);
        Utils.sleep(250);
        while(!detectedElement && opModeIsRunning()){
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
        double sign = 1;
        if(!isRed){
            sign = -1;
        }

        arm.intakeDumb(0);
    }

    public void semiDumbVisionDriving(double pow){
        while(!somethingInThePartition()){ //add the  conditional with something in the partition
            setStrafe(pow);
        }
        setEncoderMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setEncoderMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        boolean detectedElement = false;
        while(!detectedElement && opModeIsRunning()){
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
        while(!detectedElement && opModeIsRunning()){
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
                && Math.abs(data.getMotorCurrentPosition(2)) < Math.abs(blDistance) && Math.abs(data.getMotorCurrentPosition(3)) < Math.abs(brDistance)) && opModeIsRunning()){
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
        while(Math.abs(currentTicks - previousTicks) > 50 && opModeIsRunning()){
            currentTicks = systems.getCarouselPos();
            setStrafe(0.2 * direction);
            previousTicks = currentTicks;
            AbstractOpMode.currentOpMode().telemetry.addData("dc", currentTicks - previousTicks);
            AbstractOpMode.currentOpMode().telemetry.addData("why", systems.getCarouselPos());
            AbstractOpMode.currentOpMode().telemetry.update();
        }
        Utils.sleep(250);
        setStrafe(0);
        while(Math.abs(currentTicks - previousTicks) < 50 && opModeIsRunning()){
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
        while(pipeline.yPointList().get(0) < 5 && opModeIsRunning() && !eStop && !environmentalTerminate && opModeIsRunning()) { //todo ask mason how to ensure I am tracking the same cube every time here
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
        while(xPartitionDeviation > 5 && opModeIsRunning() && !eStop && !environmentalTerminate && opModeIsRunning()){
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
        strafeDistanceSensor(omega, radians, true, isRed);
    }
    public synchronized void strafeDistanceSensorOpposite(double omega, double radians){
        strafeDistanceSensor(omega, radians, true,!isRed);
    }

    public synchronized void strafeDistanceSensor(double omega, double radians, boolean isBrake, boolean isRed){


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
        double distanceFront, distanceBack;
            if(isRed) {
                distanceFrontThreshold = 3.4; //2.9
                distanceBackThreshold = 1.9; //1.4
                distanceFront = frontRed.getDistance(DistanceUnit.INCH);
                distanceBack = backRed.getDistance(DistanceUnit.INCH);

                frontRGBA = frontRed.getNormalizedColors();
                backRGBA = backRed.getNormalizedColors();
                Debug.log("red");
            }else{
                distanceFront = frontBlue.getDistance(DistanceUnit.INCH);
                distanceBack = backBlue.getDistance(DistanceUnit.INCH);

                frontRGBA = frontBlue.getNormalizedColors();
                backRGBA = backBlue.getNormalizedColors();
                Debug.log("blue");
            }

            while (distanceFront > distanceFrontThreshold  && distanceBack > distanceBackThreshold && opModeIsRunning()){

                if(isRed) {
                    frontRGBA = frontRed.getNormalizedColors();
                    backRGBA = backRed.getNormalizedColors();
                    distanceFront = frontRed.getDistance(DistanceUnit.INCH);
                    distanceBack = backRed.getDistance(DistanceUnit.INCH);
                }else{
                    frontRGBA = frontBlue.getNormalizedColors();
                    backRGBA = backBlue.getNormalizedColors();
                    distanceFront = frontBlue.getDistance(DistanceUnit.INCH);
                    distanceBack = backBlue.getDistance(DistanceUnit.INCH);
                }
                AbstractOpMode.currentOpMode().telemetry.addData("F", distanceFront);
                AbstractOpMode.currentOpMode().telemetry.addData("B", distanceBack);
                AbstractOpMode.currentOpMode().telemetry.update();

                double angle;
                if(isRed){
                    angle = 0;
                }else{
                    angle = Math.PI;
                }

                Vector2D vec = Vector2D.fromAngleMagnitude(radians + (Math.PI / 2.0) + angle, omega);
                setVelocity(vec, 0);
                Utils.sleep(100);

            }
        if(isBrake) {
            brake();
        }



    }

    public synchronized void duck(){
        double multiplier;
        if(isRed){
            multiplier = 1;
        }else{
            multiplier = -1;
        }
        setEncoderMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setEncoderMode(DcMotor.RunMode.RUN_USING_ENCODER);
        setMotorVelocity(1,1,1,1);

        systems.runCarousel(-0.2);
        double previousTics = systems.getCarouselPos();
        double deltaTics = 10;
        lowMagnitudeHardwareCycles = 0;
        double startTime = AbstractOpMode.currentOpMode().time;
        double deltaTime = AbstractOpMode.currentOpMode().time - startTime;
        while((Math.abs(deltaTics) > 10 ||  deltaTime < 1.0) && opModeIsRunning()){
            int currentTics = systems.getCarouselPos();
            deltaTics = currentTics - previousTics;
            previousTics = currentTics;
            deltaTime = AbstractOpMode.currentOpMode().time - startTime;

            AbstractOpMode.currentOpMode().telemetry.addData("delta", deltaTics);
            AbstractOpMode.currentOpMode().telemetry.addData("time", deltaTime);
            AbstractOpMode.currentOpMode().telemetry.update();
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

        while((Math.abs(newDesiredPosition.subtract(currentState.getPosition()).magnitude()) > 5.0) && opModeIsRunning() && !eStop && !environmentalTerminate){

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

     public void setMotorVelocityNew(double flVelocity, double frVelocity, double blVelocity, double brVelocity){

         fl.setVelocity(flVelocity, AngleUnit.RADIANS);
         fr.setVelocity(frVelocity , AngleUnit.RADIANS);
         bl.setVelocity(blVelocity , AngleUnit.RADIANS);
         br.setVelocity(brVelocity, AngleUnit.RADIANS);
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
        while(Math.abs(desiredRotation - state.getRotation()) > 0.05 && opModeIsRunning() && !eStop && !environmentalTerminate){
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

        while(Math.abs((state.getRotation() - radians))  > 0.05 && opModeIsRunning() && !environmentalTerminate && !eStop){
            state = localizer.getCurrentState();
//            AbstractOpMode.currentOpMode().telemetry.addData("", state);
//            AbstractOpMode.currentOpMode().telemetry.update();
            setPower(power, -power, power, -power);
        }
        brake();


    }

    public synchronized void brake() {
        fl.setVelocity(0);
        fr.setVelocity(0);
        bl.setVelocity(0);
        br.setVelocity(0);

//        fl.setPower(0);
//        fr.setPower(0);
//        bl.setPower(0);
//        br.setPower(0);
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

        double minArcLength = arclength - inchesToEncoderTicks((WHEELBASE_X / 2.0) * dAngle);
        double maxArcLength = arclength + inchesToEncoderTicks((WHEELBASE_X / 2.0) * dAngle);

        hub.clearBulkCache();
        LynxModule.BulkData data = hub.getBulkData();

        while((Math.abs(data.getMotorCurrentPosition(0)) < Math.abs(minArcLength) ||
                Math.abs(data.getMotorCurrentPosition(1)) < Math.abs(maxArcLength) ||
                Math.abs(data.getMotorCurrentPosition(2)) < Math.abs(minArcLength) ||
                Math.abs(data.getMotorCurrentPosition(3)) < Math.abs(maxArcLength)) && opModeIsRunning()) {

            AbstractOpMode.currentOpMode().telemetry.addData("fl", data.getMotorCurrentPosition(0));
            AbstractOpMode.currentOpMode().telemetry.addData("fr", data.getMotorCurrentPosition(1));
            AbstractOpMode.currentOpMode().telemetry.addData("bl", data.getMotorCurrentPosition(2));
            AbstractOpMode.currentOpMode().telemetry.addData("br", data.getMotorCurrentPosition(3));

            AbstractOpMode.currentOpMode().telemetry.addData("fl", minArcLength);
            AbstractOpMode.currentOpMode().telemetry.addData("fr", maxArcLength);

            AbstractOpMode.currentOpMode().telemetry.addData("fl", wheelMotions.getValue(0,0));
            AbstractOpMode.currentOpMode().telemetry.addData("fr", wheelMotions.getValue(1,0));
            AbstractOpMode.currentOpMode().telemetry.addData("bl", wheelMotions.getValue(2,0));
            AbstractOpMode.currentOpMode().telemetry.addData("br", wheelMotions.getValue(3,0));

            AbstractOpMode.currentOpMode().telemetry.update();

            hub.clearBulkCache();
            data = hub.getBulkData();
            setMotorVelocityNew(wheelMotions.getValue(0, 0), wheelMotions.getValue(1, 0),
                    wheelMotions.getValue(2, 0), wheelMotions.getValue(3, 0));
        }
        brake();
    }

    public synchronized void setOmniMovement(double theta, double tics, double velocity){
        setOmniMovement(theta, tics, velocity, true);
    }

    public synchronized void setOmniMovement(double theta, double tics, double velocity, boolean isBrake){
        double k2 = (1 - Math.tan(theta)) / (Math.tan(theta) + 1);
        double k1 = (k2 * ((Math.tan(theta) + 1) / (1 - Math.tan(theta)))) * getSign(k2);

        if(Math.abs(k1) > 1.0){
            k2 = k2 / k1 ;
            k1 = 1.0 ;
        }
        if(Math.abs(k2) > 1.0){
            k1 = k1 / k2 ;
            k2 = 1.0 ;
        }
        int tics1 = (int)(k1 * tics);
        int tics2 = (int)(k2 * tics);
        double v1 = k1 * velocity;
        double v2 = k2 * velocity;

//        if(Math.abs(tics1) > Math.abs(tics)){
//            tics1 = (int)tics;
//        }
//        if(Math.abs(tics2) > Math.abs(tics)){
//            tics2 = (int)tics;
//        }
//
//        if(Math.abs(v1) > Math.abs(velocity)){
//            v1 = velocity;
//        }
//        if(Math.abs(v2) > Math.abs(velocity)){
//            v2 = velocity;
//        }

        setEncoderMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setEncoderMode(DcMotor.RunMode.RUN_USING_ENCODER);
        hub.clearBulkCache();
        LynxModule.BulkData data = hub.getBulkData();
        while((Math.abs(data.getMotorCurrentPosition(0)) < Math.abs(tics1) ||
        Math.abs(data.getMotorCurrentPosition(1)) < Math.abs(tics2) ||
        Math.abs(data.getMotorCurrentPosition(2)) < Math.abs(tics1) ||
        Math.abs(data.getMotorCurrentPosition(3)) < Math.abs(tics2)) && opModeIsRunning()){
            hub.clearBulkCache();
            data = hub.getBulkData();
            setMotorVelocity(v1,v2,v1,v2);
        }
        if(isBrake) {
            brake();
        }

    }

    private final double TICKS_PER_REV = 384.5;
     private static final double WHEEL_DIAMETER = 3.78;

     public int inchesToEncoderTicks(double inches){
         return (int)((TICKS_PER_REV / (WHEEL_DIAMETER  * PI)) * inches);
     }


    /**
     * types of movement I need to splice
     * linear
     * rotation
     * color sensor localization
     * warehouse color sensor movement.
     * @param movements
     */
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
            }else if(curr.getMovement() == Movement.MovementType.PAUSE){
                brake();
                Utils.sleep(curr.getMillis());
            }else if(curr.getMovement() == Movement.MovementType.WALL_LOCALIZATION){
                strafeDistanceSensor(curr.getVelocity(), curr.getRadians(), false, isRed);
            }else if(curr.getMovement() == Movement.MovementType.WAREHOUSE_LOCALIZATION){
                driveColorSensorWarehouse(curr.getVelocity(), false);
            }else if(curr.getMovement() == Movement.MovementType.MODIFY_FLAG){
                flags[curr.getIndex()] = curr.getVal();
            }else if(curr.getMovement() == Movement.MovementType.WAREHOUSE_OPERATION){
                driveColorSensorSpliced(curr.getVelocity());
            }else if(curr.getMovement() == Movement.MovementType.ARC_MOVEMENT){
                setOmniMovement(curr.getRadians(), curr.getDistance(), curr.getVelocity(), false);
            }
        }
        brake();
    }

    public synchronized boolean getFlagIndex(int index){
        return flags[index];
    }

    public synchronized void setFlagIndex(int index, boolean val){
        flags[index] = val;
    }

    public void setEnvironmentalTerminate(boolean val){
        environmentalTerminate = val;
    }

    public void seteStop(boolean val){
        eStop = val;
    }

    public NormalizedRGBA getSensorRGBA(){
        return sensor.getNormalizedColors();
    }
}
