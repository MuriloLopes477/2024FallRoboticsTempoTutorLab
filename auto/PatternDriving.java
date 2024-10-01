package edu.elon.robotics.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import edu.elon.robotics.RobotHardware;

@Autonomous(name = "Pattern Driving")

public class PatternDriving extends AutoCommon{

    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();
        waitForStart();
//    protected RobotHardware robot;
        double power = 0.2;
        driveDistance(150, 0, power);
        sleep(500);
        driveDistance(-50, 0, power);
        sleep(500);
        turnAngle(-90, power);
        sleep(500);
        driveDistance(100, 0, power);
        sleep(500);
        turnAngle(70, power);
        sleep(500);
        driveDistance(-106.42, 0, power);
        sleep(500);
        turnAngle(-250, power);
        sleep(500);
        driveDistance(63.6, 0, power);
        sleep(500);
        turnAngle(-90, power);

    }

}
