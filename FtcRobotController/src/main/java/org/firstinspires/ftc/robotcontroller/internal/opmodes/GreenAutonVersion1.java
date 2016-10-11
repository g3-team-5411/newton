package org.firstinspires.ftc.robotcontroller.internal.opmodes;

/**
 * Created by Administrator on 10/4/2016.
 */

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.GyroSensor;
public class GreenAutonVersion1 extends OpMode {
    private DcMotor leftMotor;
    private DcMotor rightMotor;
    private final double SPEED = 0.5;
    private final double     COUNTS_PER_MOTOR_REV    = 1440 ;    // eg: TETRIX Motor Encoder
    private final double     DRIVE_GEAR_REDUCTION    = 0.5 ;     // This is < 1.0 if geared UP
    private final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    private final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    private final double WHEEL_CIRCUMFERENCE_INCHES = WHEEL_DIAMETER_INCHES * Math.PI;
    private final double AUTON_DISTANCE = 5; //TODO: change this to the correct number

    @Override
    public void init() {
        leftMotor = hardwareMap.dcMotor.get("left_drive");
        rightMotor = hardwareMap.dcMotor.get("right_drive");

//        rightMotor.setDirection(DcMotor.Direction.REVERSE);

        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    @Override
    public void start() {
        int counts = getCounts(AUTON_DISTANCE);
        leftMotor.setTargetPosition(counts);
        rightMotor.setTargetPosition(counts);

        leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftMotor.setPower(0.5);
        rightMotor.setPower(0.5);
    }

    @Override
    public void loop() {

    }

    public double distanceToRotations(double distance) {
        double rotations = distance / WHEEL_CIRCUMFERENCE_INCHES;
        return rotations;
    }

    public double rotationsToCounts(double rotations) {
        double counts = rotations * COUNTS_PER_MOTOR_REV;
        return counts;
    }

    public int getCounts(double distance) {
        int counts = (int) rotationsToCounts(distanceToRotations(distance));
        return counts;
    }

}