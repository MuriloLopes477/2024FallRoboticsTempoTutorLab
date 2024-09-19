package edu.elon.robotics.auto;

/**
 * General autonomous methods.
 */

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import edu.elon.robotics.RobotHardware;

public class AutoCommon extends LinearOpMode {

    protected RobotHardware robot;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new RobotHardware(hardwareMap, true);
    }

}



