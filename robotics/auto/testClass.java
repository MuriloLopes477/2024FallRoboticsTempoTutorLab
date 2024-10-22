package edu.elon.robotics.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="test_class", group = "labs")

public class testClass extends AutoCommon{
    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();
        waitForStart();
        driveToCalibrateLightSensor();
        int distanceDriven = driveUntilTouch(0.15);
        sleep(500);
        int distanceToTravel = (distanceDriven+20);
        telemetry.addData("Distance to travel ", distanceToTravel);
        driveIMU(distanceToTravel,0.35);
        telemetry.addData("Distance (cm) traveled from box: ", -1*distanceToTravel);
        telemetry.update();
        sleep(15000);
    }
}
