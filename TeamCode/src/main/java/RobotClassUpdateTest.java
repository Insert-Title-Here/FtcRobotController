import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robot;

import java.io.File;

@Autonomous(name="Former Kobalt Klaws now Titanium Talons Robotics java 14th edition copyright Oracle LLC Control loop parallel processed Update test the very first")
public class RobotClassUpdateTest extends LinearOpMode {
    Robot robot;
    Thread thread;
    long runtime = 20;
    File newFile;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(hardwareMap);

        thread = new Thread(){
            public void run(){
                while(opModeIsActive()){
                    long loopStart = System.currentTimeMillis();
                    robot.update();
                    long loopDelta = System.currentTimeMillis() - loopStart;
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
}
