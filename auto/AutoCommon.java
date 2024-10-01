package edu.elon.robotics.auto;

/**
 * General autonomous methods.
 */

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import edu.elon.robotics.KiwiDriveRatio;
import edu.elon.robotics.RobotHardware;

public class AutoCommon extends LinearOpMode {

    protected RobotHardware robot;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new RobotHardware(hardwareMap, true);
    }
    protected void turnAngle(double degrees, double maxPower) {
        maxPower = Math.abs(maxPower);
        robot.resetDriveEncoders();
        double ticksval = robot.convertDegreesToTicks(Math.abs(degrees));
        if (degrees > 0) {
            robot.startMove(0,0, maxPower);
        }
        else {
            robot.startMove(0,0,maxPower*-1);
        }
        while (opModeIsActive() && Math.abs(robot.motorLeft.getCurrentPosition()) < ticksval) {}

        robot.startMove(0,0,0);


        //be able to turn 90 degrees
        //reset the encoders
        /*
        -convert degrees to ticks
        -start motor at magnitude max power
        -while loop until encoder reaches the amount of ticks
        -stop motor


         */
    }


    protected void driveDistance(double cmForward, double cmSide, double maxPower) {

        double drive = -1*cmForward;
        double strafe = cmSide;

        KiwiDriveRatio ratio = new KiwiDriveRatio(true);
        ratio.computeRatio(drive, strafe, 0);

        double distance_cm = Math.sqrt(Math.pow(cmForward,2) + Math.pow(cmSide,2));


        DcMotor motorToWatch = robot.motorRight;
        double radsFromCenter = Math.toRadians(37);
       if (Math.abs(ratio.powerLeft) > Math.abs(ratio.powerRight) && Math.abs(ratio.powerLeft) > Math.abs(ratio.powerAux)  ) {
           motorToWatch = robot.motorLeft;
           radsFromCenter = Math.toRadians(-37);
       } else if (Math.abs(ratio.powerAux) > Math.abs(ratio.powerRight) && Math.abs(ratio.powerAux) > Math.abs(ratio.powerLeft)  ) {
           motorToWatch = robot.motorAux;
           radsFromCenter = Math.toRadians(0);
       }
       drive *= (maxPower/distance_cm);
       strafe *= (maxPower/distance_cm);

       int computedStopTicks = robot.convertDistanceToTicks(distance_cm);

       double adjustment = 1.0;
        int stopTicks = 0;
        int stopTicks2 = 0;
            stopTicks = (int) (robot.convertDistanceToTicks(distance_cm / (Math.cos(radsFromCenter)*1.203660967)));
            telemetry.addData("strafing: ", cmSide + " with updatedTicks valued at " + stopTicks + " ticks");
            telemetry.update();

            stopTicks2 = (int) (robot.convertDistanceToTicks(distance_cm / (Math.sin(radsFromCenter)*2)));
            stopTicks = Math.min(stopTicks2,stopTicks);
            telemetry.addData("Driving: ", cmForward + " with updatedTicks valued at " + stopTicks + " ticks\n " +
                    "strafing: ", cmSide + " with updatedTicks valued at " + stopTicks + " ticks");
            telemetry.update();
//           adjustment = Math.abs(Math.cos(radsFromCenter - Math.atan(cmSide/cmForward)));
//           stopTicks =(int) (computedStopTicks * adjustment);


//       else
           /*if (cmForward != 0) {
           adjustment = Math.abs(Math.cos(radsFromCenter - (Math.cos(Math.abs(cmSide))/Math.sin(Math.abs(cmForward))) ));
       }*/


        robot.resetDriveEncoders(); //zeros encoders
        robot.startMove(drive,strafe,0);
        /*double accelerationFactorDrive = 0.0;
        double accelerationFactorStrafe = 0.0;
       */while (opModeIsActive() && Math.abs(motorToWatch.getCurrentPosition()) < stopTicks) {
            /*if (Math.abs(motorToWatch.getCurrentPosition()) < (int) (stopTicks/4)) {
                robot.startMove(drive*accelerationFactorDrive,strafe*accelerationFactorStrafe,0.0);
                accelerationFactorDrive = (Math.abs(accelerationFactorDrive) > 1)? 1: accelerationFactorDrive + (drive * 4/stopTicks);
                accelerationFactorStrafe = (Math.abs(accelerationFactorStrafe) > 1)? 1: accelerationFactorStrafe + (strafe * 4/stopTicks);
            } else if (Math.abs(motorToWatch.getCurrentPosition()) > (int) 3*(stopTicks/4)) {
                robot.startMove(drive*accelerationFactorDrive,strafe*accelerationFactorStrafe,0.0);
                accelerationFactorDrive = (accelerationFactorDrive > 1)? 1: accelerationFactorDrive - (drive * 4/stopTicks);
                accelerationFactorStrafe = (accelerationFactorStrafe > 1)? 1: accelerationFactorStrafe - (strafe * 4/stopTicks);
            }*/
       }
       robot.startMove(0,0,0);
    }
}



