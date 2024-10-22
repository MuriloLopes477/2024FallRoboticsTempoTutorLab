package edu.elon.robotics.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import edu.elon.robotics.RobotHardware;

@Autonomous(name="EncoderDrive", group = "labs")
public class EncoderDrive extends AutoCommon {

    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();
        waitForStart();

        turnAngle(90, 0.3);
        sleep(1000);
        turnAngle(-90, 0.3);
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

    public void turningRobot() {
        robot.resetDriveEncoders(); //zeros encoders
        robot.startMove(0,0,0.3);
        while (opModeIsActive() && Math.abs(robot.motorAux.getCurrentPosition()) < RobotHardware.TICKS_PER_ROTATION * 3) {
            // nothing here for now
        }
        robot.startMove(0,0,0.0);

    }


}
