package edu.elon.robotics.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import edu.elon.robotics.auto.AutoCommon;

@Autonomous(name="imu Test", group = "labs")

public class IMUTest extends AutoCommon {
    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();
        waitForStart();

        while (opModeIsActive()) {
            // You need to call this to fill the robotOrientation object
            // This needs to be done each time you want a new reading
            // which is why it is called inside the loop.
            // It will not update by itself
            YawPitchRollAngles robotOrientation = robot.imu.getRobotYawPitchRollAngles();

            // Then use these methods to extract each angle that you want
            // from the robotOrientation object.
            // This needs to be done each time you want a new reading and after
            //    robotOrientation = robot.imu.getRobotYawPitchRollAngles();
            // They will not update by themselves
            double yaw   = robotOrientation.getYaw(AngleUnit.DEGREES);
            double pitch = robotOrientation.getPitch(AngleUnit.DEGREES);
            double roll  = robotOrientation.getRoll(AngleUnit.DEGREES);

            telemetry.addData("roll", roll);
            telemetry.addData("pitch", pitch);
            telemetry.addData("yaw", yaw);
            telemetry.update();
        }
    }
}