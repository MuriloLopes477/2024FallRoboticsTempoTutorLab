package edu.elon.robotics;

/**
 * Defines the robot hardware and implements a few
 * fundamental methods.
 */

import android.media.MediaPlayer;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.robot.Robot;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

public class RobotHardware {

    // drive motors
    public DcMotor motorLeft;
    public DcMotor motorRight;
    public DcMotor motorAux;
    public IMU imu;
    private KiwiDriveRatio ratio;

    public static final double TICKS_PER_ROTATION = 537.7;
    public final double WHEEL_CIRCUMFERENCE = 29.1593; // in centimeters
    public final double TICKS_PER_CM = TICKS_PER_ROTATION / WHEEL_CIRCUMFERENCE;
    public final double TURNING_DIAMETER = 2 * 100* 0.1351665212;
    public final double ROBOT_CIRCUMFERENCE = TURNING_DIAMETER * Math.PI;

    // sensors
    public RevTouchSensor touchSensor;
    public ColorSensor colorSensor;

    //color sensor brightness levels
    public int maxBrightness;
    public int minBrightness;

    //color sensor brightness levels
    public int maxRed;
    public int minRed;

    public RobotHardware(HardwareMap hardwareMap, boolean isAuto) {

        // external sensors
        touchSensor = hardwareMap.get(RevTouchSensor.class, "touchSensor");
        colorSensor = hardwareMap.get(ColorSensor.class, "colorSensor");

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
        /*
         * Define the orientation of the Control (and the IMU inside).
         */

        // the logo on the control hub is pointed up toward the sky
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection =
                RevHubOrientationOnRobot.LogoFacingDirection.UP;

        // the usb on the control hub is pointed up toward the forward
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  =
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;

        // set this orientation
        RevHubOrientationOnRobot orientationOnRobot = new
                RevHubOrientationOnRobot(logoDirection, usbDirection);

        // now initialize the IMU with this mounting orientation
        // this assumes the IMU to be in a REV Control Hub is named "imu"
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(orientationOnRobot));

// define the current direction as 0
        imu.resetYaw();
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
    public double convertTicksToDistance(int ticks) {
        return (double) (ticks * (1/TICKS_PER_CM));
    }
    public int convertDegreesToTicks(double degrees) {

        double arcLength = (degrees/360.0) * ROBOT_CIRCUMFERENCE;
        return convertDistanceToTicks(arcLength);

    }
    public double getHeading(){
        YawPitchRollAngles robotOrientation = imu.getRobotYawPitchRollAngles();
        return robotOrientation.getYaw(AngleUnit.DEGREES);//(degrees < 0)? 180 + Math.abs(degrees): degrees;
    }


}

