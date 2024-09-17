package edu.elon.robotics.teleop;

/**
 * Manually drive the robot using the game controller.
 */

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import edu.elon.robotics.RobotHardware;

@TeleOp(name = "Drive Robot", group = "TeleOp")
public class DriveRobot extends LinearOpMode {

    /*
     * Declare a variable that will represent the
     * robot hardware (i.e., the robot).
     */
    private RobotHardware robot;

    @Override
    public void runOpMode() throws InterruptedException {

        // instantiate the robot (pass the hardware configuration)
        robot = new RobotHardware(hardwareMap, false);

        // block and wait until start is pressed
        waitForStart();

        while (opModeIsActive()){
            stickDriving();
            telemetry.update();
        }

    }

    public void stickDriving() {
        /*
         * Read the gamepad joysticks and use that information
         * to drive the robot.
         */
        double drive  = -gamepad1.left_stick_y;
        double strafe = -gamepad1.left_stick_x;
        double turn   = gamepad1.right_stick_x;

        /*
         * Telemetry shows up at the bottom of the
         * drive station. It's a good way to help
         * you debug your code.
         */
        telemetry.addData("drive", drive);
        telemetry.addData("strafe", strafe);
        telemetry.addData("turn", turn);

        // call startMove to move the robot
        robot.startMove(drive, strafe, turn);
    }

}
