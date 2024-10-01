package edu.elon.robotics.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "TestTurning")
public class TestTurning extends AutoCommon {

    private final double[]  ANGLE_SEQ = {90, -90, -180, 180, 45, -45,
            90, 90, 90, 90, -90, -90, -90, -90};
    private final double   SLOW_SPEED = 0.3;
    private final double MEDIUM_SPEED = 0.65;
    private final double   FAST_SPEED = 1.0;
    private final long    SHORT_PAUSE = 250;
    private final long     LONG_PAUSE = 3000;

    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();
        waitForStart();

        // do slow speed turning sequence
        for (double angle : ANGLE_SEQ) {
            telemetry.addData("Turning", angle + " at " + SLOW_SPEED + " speed");
            telemetry.update();
            turnAngle(angle, SLOW_SPEED);
            if (!opModeIsActive()) return;
            sleep(SHORT_PAUSE);
        }

        // take a break
        sleep(LONG_PAUSE);

        // do medium speed turning sequence
        for (double angle : ANGLE_SEQ) {
            telemetry.addData("Turning", angle + " at " + MEDIUM_SPEED + " speed");
            telemetry.update();
            turnAngle(angle, MEDIUM_SPEED);
            if (!opModeIsActive()) return;
            sleep(SHORT_PAUSE);
        }

        // take a break
        sleep(LONG_PAUSE);

        // do fast speed turning sequence
        for (double angle : ANGLE_SEQ) {
            telemetry.addData("Turning", angle + " at " + FAST_SPEED + " speed");
            telemetry.update();
            turnAngle(angle, FAST_SPEED);
            if (!opModeIsActive()) return;
            sleep(SHORT_PAUSE);
        }
    }

}