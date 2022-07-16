import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.Robot;

import java.io.File;
import java.io.FileNotFoundException;
import java.io.PrintStream;

@Autonomous(name="Former Kobalt Klaws now Titanium Talons Robotics java 14th edition copyright Oracle LLC Control loop parallel processed Update test the very first")
public class RobotClassUpdateTest extends LinearOpMode {
    Robot robot;
    Thread thread;
    long runtime = 20; //calibrate this

    /*
    IMU tuning
    PID Control
    drift elimination (pid)
    OpenCV?
     */

    //logging params
    File logFile;
    String logString;
    double iterator;
    double sum;
    long max, min;

    @Override
    public void runOpMode() throws InterruptedException {
        logFile = AppUtil.getInstance().getSettingsFile("RuntimeCalibration.txt");
        logString = "";
        iterator = 0;
        sum = 0;
        max = Long.MIN_VALUE;
        min = Long.MAX_VALUE;

        robot = new Robot(hardwareMap);
        thread = new Thread(){
            public void run(){
                while(opModeIsActive() && !isStopRequested()){
                    long loopStart = System.currentTimeMillis();
                    robot.update();
                    long loopDelta = System.currentTimeMillis() - loopStart;
                    if(loopDelta > max){
                        max = loopDelta;
                    }
                    if(loopDelta < min){
                        min = loopDelta;
                    }
                    sum += loopDelta;
                    logString += loopDelta + ", " + iterator + "\n";
                    iterator++;
                    if(loopDelta <= runtime){
                        try {
                            Thread.currentThread().sleep(runtime - loopDelta);
                        } catch (InterruptedException e) {
                            e.printStackTrace();
                        }
                    }else{
                        //memory leaks are bad
                    }

                }

            }
        };
        waitForStart();
        startOpMode();
    }

    public void startOpMode(){
        thread.start();
        while(opModeIsActive()){
            telemetry.addData("average", sum / iterator);
            telemetry.addData("max", max);
            telemetry.addData("min", min);
            telemetry.update();

        }

        logString += "CALCULATED VALUES";
        logString += "sum: " + (sum / iterator) + "\n";
        logString += "max: " + max + "\n";
        logString += "min: " + min + "\n\n" ;



        try {
            PrintStream ps = new PrintStream(logFile);
            ps.println(logString);
            ps.close();
        } catch (FileNotFoundException e) {
            e.printStackTrace();
        }
    }
}
