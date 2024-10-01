package edu.elon.robotics;

/**
 * Defines the robot hardware and implements a few
 * fundamental methods.
 */

import android.media.MediaPlayer;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

public class RobotHardware {

    // drive motors
    public DcMotor motorLeft;
    public DcMotor motorRight;
    public DcMotor motorAux;

    private KiwiDriveRatio ratio;

    public static final double TICKS_PER_ROTATION = 537.7;
    public final double WHEEL_CIRCUMFERENCE = 29.1593; // in centimeters
    public final double TICKS_PER_CM = TICKS_PER_ROTATION / WHEEL_CIRCUMFERENCE;
    public final double TURNING_DIAMETER = 2 * 100* 0.1351665212;
    public final double ROBOT_CIRCUMFERENCE = TURNING_DIAMETER * Math.PI;



    public RobotHardware(HardwareMap hardwareMap, boolean isAuto) {


        /*
         * This code provides an object to control the physical
         * motor referenced by the configuration string.  The
         * positive direction of rotation is established. Finally,
         * the motor is directed to forcefully stop when no power
         * is applied.
         */

        // define the drive motors
        motorLeft = hardwareMap.dcMotor.get("motorLeft");
//        motorLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        motorLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        motorLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motorRight = hardwareMap.dcMotor.get("motorRight");
//        motorRight.setDirection(DcMotorSimple.Direction.REVERSE);
        motorLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        motorRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motorAux = hardwareMap.dcMotor.get("motorAux");
//        motorAux.setDirection(DcMotorSimple.Direction.REVERSE);
        motorLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        motorAux.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // reset the drive encoders to zero
        resetDriveEncoders();

        ratio = new KiwiDriveRatio(isAuto);

    }

    public void resetDriveEncoders() {
        /*
         * This code resets the encoder values back to 0 for
         * each of the three drive motors.
         */
        motorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        motorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        motorAux.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorAux.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void startMove(double drive, double strafe, double turn, double modifier) {
        /*
         * How much power should we apply to the left,
         * right, and aux motor?
         *
         * If all 3 motors apply the same power in the
         * same direction, the robot will turn in place.
         */
        ratio.computeRatio(drive, strafe, turn);


        modifier = Range.clip(modifier,0.0,1.0);

        /*
         * Apply the power to the motors.
         */
        motorLeft.setPower(ratio.powerLeft   * modifier);
        motorRight.setPower(ratio.powerRight * modifier);
        motorAux.setPower(ratio.powerAux     * modifier);
    }
    public void startMove(double drive, double strafe, double turn) {
        startMove(drive, strafe, turn, 1);
    }
    public int convertDistanceToTicks(double cm) {
        return (int) (cm * TICKS_PER_CM);
    }
    public int convertDegreesToTicks(double degrees) {

        double arcLength = (degrees/360.0) * ROBOT_CIRCUMFERENCE;
        return convertDistanceToTicks(arcLength);

    }
}
