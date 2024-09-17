package edu.elon.robotics.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="TimedDrive", group = "labs")
public class EncoderDrive extends AutoCommon {

    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();
        waitForStart();

        robot.resetDriveEncoders(); //zeros encoders
        robot.startMove(0.3,0.3,0.0);
        while (opModeIsActive() && Math.abs(robot.motorLeft.getCurrentPosition()) < 1074) {
            // nothing here for now
        }
        robot.startMove(0.0,0.0,0.0);

        System.out.println("MYDATA: " +
                robot.motorLeft.getCurrentPosition() + ","+
                robot.motorRight.getCurrentPosition() + ","+
                robot.motorAux.getCurrentPosition());

    }

}
