package edu.elon.robotics.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import edu.elon.robotics.KiwiDriveRatio;

@Autonomous(name = "Test Driving")
public class TestDriving extends AutoCommon {

    private final double[]  DRIVE_SEQ = {100, -100,   0,    0, 75, -75,  -50,   50, 100, -100};
    private final double[] STRAFE_SEQ = {  0,    0, 100, -100, 75, -75,  100, -100, -50,   50};
    private final double   SLOW_SPEED = 0.3;
    private final double MEDIUM_SPEED = 0.65;
    private final double   FAST_SPEED = 1.0;
    private final long    SHORT_PAUSE = 250;
    private final long     LONG_PAUSE = 1000;

    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();
        waitForStart();

        // do slow speed driving sequence
        for (int i = 0; i < DRIVE_SEQ.length; i++) {

            driveDistance(DRIVE_SEQ[i], STRAFE_SEQ[i], SLOW_SPEED);
            if (!opModeIsActive()) return;
            sleep(LONG_PAUSE);
        }

        // take a break
        sleep(LONG_PAUSE);

        // do slow speed driving sequence
        for (int i = 0; i < DRIVE_SEQ.length; i++) {
            telemetry.addData("Driving", DRIVE_SEQ[i] + "cm forward "
                    + STRAFE_SEQ[i] + "cm strafe at " + MEDIUM_SPEED + " speed");
            telemetry.update();
            driveDistance(DRIVE_SEQ[i], STRAFE_SEQ[i], MEDIUM_SPEED);
            if (!opModeIsActive()) return;
            sleep(SHORT_PAUSE);
        }

        // take a break
        sleep(LONG_PAUSE);

        // do slow speed driving sequence
        for (int i = 0; i < DRIVE_SEQ.length; i++) {
            telemetry.addData("Driving", DRIVE_SEQ[i] + "cm forward "
                    + STRAFE_SEQ[i] + "cm strafe at " + FAST_SPEED + " speed");
            telemetry.update();
            driveDistance(DRIVE_SEQ[i], STRAFE_SEQ[i], FAST_SPEED);
            if (!opModeIsActive()) return;
            sleep(SHORT_PAUSE);
        }
    }

}