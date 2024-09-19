package edu.elon.robotics.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

//@Autonomous(name="TimedDrive", group = "labs")
public class TimedDrive extends AutoCommon {
    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();

        waitForStart();
        //driveForTime(0.8,2500);
    }
    protected void driveForTime(double power, long milliseconds) {
        robot.startMove(power, 0, 0.6, 0);
        sleep(milliseconds/4);
        robot.startMove(-power, 0, 0.6, 0);
        sleep(milliseconds/2);
        robot.startMove(power, 0, 0.6, 0);
        sleep(milliseconds/4);
        robot.startMove(0, 0, 0, 0);

    }

}
