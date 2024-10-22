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
    // how many degrees to adjust the requested degree angle by
    private final double ANGLE_OVERSHOOT = 5.0;

    // slow power of the motor for the final part of the turn
    private final double TURN_ENDING_POWER = 0.12;

    // number of degrees that will be done using the slow power
    private final double SLOW_DOWN_DEGREES = 10.0;


    @Override
    public void runOpMode() throws InterruptedException {
        robot = new RobotHardware(hardwareMap, true);
        robot.imu.resetYaw();
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

    protected void turnIMU(double degrees, double power) {
        robot.imu.resetYaw();
        power = Math.abs(power);
        double h = -4.6;
        double E =  Math.abs(degrees);
        double sign = Math.signum(degrees);
        double currentHeading = 0.0;
        degrees = degrees % 360.0;
        degrees = (Math.abs(degrees) > 180)? -1*(180 - (Math.abs(degrees) % 180)): degrees;
        sign = sign * Math.signum(degrees);
//        robot.startMove(0,0,(Math.signum(degrees)*power*sign));
        double Maxpower = power;
        while (opModeIsActive() && Math.abs(currentHeading) < (Math.abs(degrees) - ANGLE_OVERSHOOT)) {

//            robot.imu.resetYaw();
//            currentHeading += Math.abs((Math.abs(robot.getHeading()) - (currentHeading%180)));
            //currentHeading += Math.abs(robot.getHeading());
            //robot.imu.resetYaw();
//            telemetry.addData("Cur-hed: ",currentHeading);
//            telemetry.addData("power: ",power);
//            telemetry.addData("num: ", (Math.pow(currentHeading,h)*Maxpower));
//            telemetry.addData("den1: ", (Math.pow((currentHeading),h)));
//            telemetry.addData("den2: ", Math.pow(E,h));


//           Failed attempt
            currentHeading = Math.abs(robot.getHeading());
            power = (currentHeading ==0.0)? Maxpower: ((Math.pow(currentHeading,h)*Maxpower)/(Math.pow((currentHeading),h) + Math.pow(E,h)));

            robot.startMove(0,0,sign*power);
        }
        telemetry.update();
        robot.startMove(0,0,0);
    }

    protected void turnIMUKeepOthers(double degrees, double power, double drive, double strafe) {
        robot.imu.resetYaw();
        power = Math.abs(power);

        double h = -4.6;
        double E =  Math.abs(degrees);
        double sign = Math.signum(degrees);
        double currentHeading = 0.0;

        degrees = degrees % 360.0;
        degrees = (Math.abs(degrees) > 180)? -1*(180 - (Math.abs(degrees) % 180)): degrees;
        sign = sign * Math.signum(degrees);

        double Maxpower = power;
        while (opModeIsActive() && Math.abs(currentHeading) < (Math.abs(degrees) - 0.5)) {

            currentHeading = Math.abs(robot.getHeading());
            power = (currentHeading ==0.0)? Maxpower: ((Math.pow(currentHeading,h)*Maxpower)/(Math.pow((currentHeading),h) + Math.pow(E,h)));
            robot.startMove(drive,strafe,sign*power);
        }
        robot.startMove(drive,strafe,0);

    }

    protected void driveIMU(double cm, double power) {

        double KP = 0.025;

        DcMotor motorToWatch = robot.motorRight;
        double radsFromCenter = Math.toRadians(37);


        int stopTicks = 0;
        int stopTicks2 = 0;
        stopTicks = (int) (robot.convertDistanceToTicks(Math.abs(cm) / (Math.cos(radsFromCenter)*1.203660967)));

        stopTicks2 = (int) (robot.convertDistanceToTicks(Math.abs(cm) / (Math.sin(radsFromCenter)*2)));
        stopTicks = (int) ((stopTicks2+stopTicks)/1.95);

        robot.resetDriveEncoders(); //zeros encoders
        robot.startMove(power* Math.signum(cm),0,0);

        double difference;

        robot.imu.resetYaw();
       while (opModeIsActive() && Math.abs(motorToWatch.getCurrentPosition()) < stopTicks) {
            difference = robot.getHeading();
            robot.motorAux.setPower(difference * KP);
       }
        robot.startMove(0,0,0);
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



        robot.resetDriveEncoders(); //zeros encoders
        robot.startMove(drive,strafe,0);

       while (opModeIsActive() && Math.abs(motorToWatch.getCurrentPosition()) < stopTicks) {

       }
       robot.startMove(0,0,0);
    }

    protected int driveUntilTouch(double power) {
        power *= -1;
        double radsFromCenter = Math.toRadians(37);
        robot.resetDriveEncoders();
        robot.startMove(power, 0, 0);
        boolean counted = false;
        int lineCount = 0;
        int lineCountRed = 0;
        int lineCountWhite = 0;
        double lineWidth = 0;
        int lineTicksBeginning = 0;
        int lineTicksEnd = 0;
        DcMotor MotorToWatch = robot.motorRight;
        String output = "";
        String lineColor = "";
        while (opModeIsActive() && !robot.touchSensor.isPressed()) {
            if (!counted && ((robot.colorSensor.alpha() > ((robot.maxBrightness - robot.minBrightness) / 2)))) {
                //|| (robot.colorSensor.red() > ((robot.maxRed - robot.minRed) /2.5)))) {
//                if (robot.colorSensor.alpha() > ((robot.maxBrightness - robot.minBrightness) /2) ) {
//                    lineColor = "white";
//                    lineCountWhite++;
//                }
//                } else if (robot.colorSensor.red() > ((robot.maxRed - robot.minRed) /2.4) ) {
//                    lineColor = "red";
//                    lineCountRed++;
//                }
                counted = true;
                lineCount++;
                lineTicksBeginning = MotorToWatch.getCurrentPosition();

            } else if (robot.colorSensor.alpha() < ((robot.maxBrightness - robot.minBrightness) / 2)) { //&& !(robot.colorSensor.red() > ((robot.maxRed - robot.minRed) / 2.4))) {
//                if (counted) {
//                    lineTicksEnd = MotorToWatch.getCurrentPosition();
//                    lineWidth = robot.convertTicksToDistance(lineTicksEnd - lineTicksBeginning) * Math.sin(radsFromCenter)*2;
//                    output += "Line number: " + lineCount + "\n";
//                    output += "Line width (sin): " + lineWidth + "\n";
//                    lineWidth = robot.convertTicksToDistance(lineTicksEnd - lineTicksBeginning) * Math.cos(radsFromCenter)*2;
//                    output += "Line width (cos): " + lineWidth + "\n";
//                }
                counted = false;


            }
        }

//        telemetry.addData("output:\n ", output);
        telemetry.addData("Line count total: ", lineCount);
//        telemetry.addData("Red line count: ", lineCountRed);
//        telemetry.addData("White line count: ", lineCountWhite);
        robot.startMove(0, 0, 0);
        return (int) Math.abs(robot.convertTicksToDistance(robot.motorRight.getCurrentPosition()));
    }

    protected void driveToCalibrateLightSensor() {
        robot.colorSensor.enableLed(true);

        int cm = 20;
        double power = 0.4;
        double KP = 0.025;
        double drive = -1*cm;

        DcMotor motorToWatch = robot.motorRight;
        double radsFromCenter = Math.toRadians(37);
        drive *= (power/cm);

        int stopTicks = 0;
        int stopTicks2 = 0;
        stopTicks = (int) (robot.convertDistanceToTicks(cm / (Math.cos(radsFromCenter)*1.203660967)));
        stopTicks2 = (int) (robot.convertDistanceToTicks(cm / (Math.sin(radsFromCenter)*2)));
        stopTicks = Math.min(stopTicks2,stopTicks);
        robot.resetDriveEncoders(); //zeros encoders
        robot.startMove(drive,0,0);

        double difference;

        robot.imu.resetYaw();

        int currentAlpha = robot.colorSensor.alpha();
        int maxAlpha = currentAlpha;
        int minAlpha = currentAlpha;
        int currentRed = robot.colorSensor.red();
        int minRed = currentRed;
        int maxRed = currentRed;
        while (opModeIsActive() && Math.abs(motorToWatch.getCurrentPosition()) < stopTicks) {
            difference = robot.getHeading();
            //telemetry.addData("difference: ", difference);
            robot.motorAux.setPower(difference * KP);

            currentAlpha = robot.colorSensor.alpha();
            maxAlpha = Math.max(currentAlpha, maxAlpha);
            minAlpha = Math.min(currentAlpha, minAlpha);


            currentRed = robot.colorSensor.red();
            maxRed = Math.max(currentRed, maxRed);
            minRed = Math.min(currentRed, minRed);
        }
       robot.startMove(0,0,0);
//       telemetry.addData("max-alpha", maxAlpha);
//       telemetry.addData("min-alpha", minAlpha);
        robot.maxBrightness = maxAlpha;
        robot.minBrightness = minAlpha;
        robot.maxRed = maxRed;
        robot.minRed = minRed;

//       telemetry.update();
    }


    protected void collectData() {
        robot.colorSensor.enableLed(true);

        while (true) {

            if (robot.colorSensor.alpha() > ((robot.maxBrightness - robot.minBrightness) /2) ) {
                telemetry.addData("White line", robot.colorSensor.alpha());
                telemetry.addData("White line", robot.colorSensor.red());
                telemetry.update();
            } else if (robot.colorSensor.red() > ((robot.maxRed - robot.minRed) /2.5) ) {
                telemetry.addData("Red Line", robot.colorSensor.alpha());
                telemetry.addData("Red line", robot.colorSensor.red());
                telemetry.update();
            } else {
                telemetry.addData("Black background", robot.colorSensor.alpha());
                telemetry.addData("Black background", robot.colorSensor.red());
                telemetry.update();
            }

        }
    }

}



