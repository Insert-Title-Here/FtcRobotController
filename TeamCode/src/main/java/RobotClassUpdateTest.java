import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.OpModeWrapper;
import org.firstinspires.ftc.teamcode.Robot;

import java.io.File;
import java.io.FileNotFoundException;
import java.io.PrintStream;
import java.util.concurrent.atomic.AtomicBoolean;

@Autonomous(name="Former Kobalt Klaws now Titanium Talons Robotics java 14th edition copyright Oracle LLC Control loop parallel processed Update test the very first")
public class RobotClassUpdateTest extends OpModeWrapper {
    Robot robot;
    Thread thread;
    long runtime = 20; //calibrate this

    /*
    IMU tuning
    PID Control
    drift elimination (pid)
    OpenCV?
    thread instance
     */

    //logging params
    File logFile;
    String logString;
    double iterator;
    double sum;
    long max, min;
    AtomicBoolean shouldUpdate;




    @Override
    protected void onInitialize() throws FileNotFoundException {
        logFile = AppUtil.getInstance().getSettingsFile("RuntimeCalibration.txt");
        logString = "";
        iterator = 0;
        sum = 0;
        max = Long.MIN_VALUE;
        min = Long.MAX_VALUE;
        shouldUpdate = new AtomicBoolean(true);

        robot = new Robot(hardwareMap);
        thread = new Thread(){
            public void run(){
                while(shouldUpdate.get()){
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
    }

    @Override
    protected void onStart() {
        thread.start();
        while(opModeIsActive()){
            telemetry.addData("average", sum / iterator);
            telemetry.addData("max", max);
            telemetry.addData("min", min);
            telemetry.update();

        }

    }

    @Override
    protected void onStop() {
        shouldUpdate.set(false);
        robot.stop();
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
