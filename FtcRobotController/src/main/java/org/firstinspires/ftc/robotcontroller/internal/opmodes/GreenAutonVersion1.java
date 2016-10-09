package org.firstinspires.ftc.robotcontroller.internal.opmodes;

/**
 * Created by Administrator on 10/4/2016.
 */

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.GyroSensor;
public class GreenAutonVersion1 extends OpMode {

    DcMotor leftMotor;
    DcMotor rightMotor;
    final double SPEED = 0.5;
    static final double     COUNTS_PER_MOTOR_REV    = 1440 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 0.5 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double WHEEL_CIRCUMFERENCE_INCHES = WHEEL_DIAMETER_INCHES * Math.PI;

    @Override
    public void init() {
        leftMotor = hardwareMap.dcMotor.get("left_drive");
        rightMotor = hardwareMap.dcMotor.get("right_drive");

//        rightMotor.setDirection(DcMotor.Direction.REVERSE);

        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    }

    @Override
    public void loop() {
        driveForward(24);
    }

    public double distanceToRotations(double distance) {
        double rotations = distance / WHEEL_CIRCUMFERENCE_INCHES;
        return rotations;
    }

    public double rotationsToCounts(double rotations) {
        double counts = rotations * COUNTS_PER_MOTOR_REV;
        return counts;
    }

    public void driveForward(double distance) {

    }

    public void rightTurn(double degrees) {
        leftMotor.setPower(SPEED);
        rightMotor.setPower(0);
    }

    public void leftTurn(double degrees) {
        leftMotor.setPower(0);
        rightMotor.setPower(SPEED);
    }

    public void stopRobot() {
        leftMotor.setPower(0);
        rightMotor.setPower(0);
    }
}