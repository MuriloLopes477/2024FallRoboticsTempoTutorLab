package edu.elon.robotics.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import edu.elon.robotics.RobotHardware;

@Autonomous(name="TimedDrive", group = "labs")
public class EncoderDrive extends AutoCommon {

    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();
        waitForStart();

        for (int i = 1; i <= 12; i++) {
            robotTestRun(robot, i*50, 0.0);
            sleep(10000);
        }

        System.out.println("MYDATA: " +
                robot.motorLeft.getCurrentPosition() + ","+
                robot.motorRight.getCurrentPosition() + ","+
                robot.motorAux.getCurrentPosition());

    }

    public void robotTestRun(RobotHardware robot, int tickCount, double pow) {
        robot.resetDriveEncoders(); //zeros encoders
        robot.startMove(pow,0.3,0.0);
        while (opModeIsActive() && Math.abs(robot.motorAux.getCurrentPosition()) < tickCount) {
            // nothing here for now
        }
        robot.startMove(0.0,0,0.0);
        sleep(5000);
        int cur = Math.abs(robot.motorAux.getCurrentPosition());
        while (cur > 5) {
            robot.startMove(-1*pow,-1*0.3,0.0);
            cur = Math.abs(robot.motorAux.getCurrentPosition());

        }

        robot.startMove(0.0,0.0,0.0);
    }

}
