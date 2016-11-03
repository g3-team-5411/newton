package org.firstinspires.ftc.robotcontroller.internal.opmodes;

/**
 * Created by Administrator on 10/4/2016.
 */




import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.GyroSensor;


public class BlueCenterPark extends LinearOpMode {

    private DcMotor leftMotor;
    private DcMotor rightMotor;
    private final double SPEED = 0.25;
    private final double COUNTS_PER_MOTOR_REV = 1440;    // eg: TETRIX Motor Encoder
    private final double DRIVE_GEAR_REDUCTION = 0.5;     // This is < 1.0 if geared UP
    private final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    private final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    private final double WHEEL_CIRCUMFERENCE_INCHES = WHEEL_DIAMETER_INCHES * Math.PI;

    private final double AUTON_DISTANCE = 50; //TODO: change this to the correct number
    private GyroSensor gyroSensor;
    private final double TURN_DEGREES = -45;

    @Override
    public void runOpMode() throws InterruptedException {

        leftMotor = hardwareMap.dcMotor.get("left_drive");
        rightMotor = hardwareMap.dcMotor.get("right_drive");
        gyroSensor = hardwareMap.gyroSensor.get("gyro");
        rightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        calibrateGyro();
        waitForStart();
        telemetry.addData("STARTED", "true");
        telemetry.update();
        driveStraight(AUTON_DISTANCE);
        sleep(5000);
        turn(TURN_DEGREES);
        sleep(10000);
        // driveStraight(AUTON_DISTANCE);

    }
    public void driveStraight(double distance){
        telemetry.addData("Driving Straight", "true");
        telemetry.update();
        double counts = getCounts(distance);
        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftMotor.setTargetPosition((int) counts);
        rightMotor.setTargetPosition((int) counts);
        leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftMotor.setPower(0.5);
        rightMotor.setPower(0.5);
        telemetry.addData("Driving Straight", "false");
        telemetry.update();
    }
//    @Override
//    public void init() {
//        leftMotor = hardwareMap.dcMotor.get("left_drive");
//        rightMotor = hardwareMap.dcMotor.get("right_drive");
//        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
////      rightMotor.setDirection(DcMotor.Direction.REVERSE);
//
//    }





//    @Override
//    public void start() {
//        double counts = getCounts(AUTON_DISTANCE);
//        leftMotor.setTargetPosition((int) counts);
//        rightMotor.setTargetPosition((int) counts);
//        leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        leftMotor.setPower(0.5);
//        rightMotor.setPower(0.5);
//        leftMotor.setPower(0.5);
//        rightMotor.setPower(0.0);
//    }
//
//    @Override
//    public void loop() {
//        driveForward(24);
//
//    }



    public void driveForward(double distance) {
        leftMotor.setPower(SPEED);
        rightMotor.setPower(SPEED);

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


    public void turn(double targetDegrees) {
        double currentDegrees = gyroSensor.getHeading();
        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        while (Math.abs(currentDegrees) < Math.abs(targetDegrees)){
            currentDegrees = gyroSensor.getHeading();
            leftMotor.setPower(.5);
            rightMotor.setPower(0);
            telemetry.addData("an attempt was made", true);
            telemetry.addData("Current Degrees", currentDegrees);
            telemetry.addData("Target Degrees", targetDegrees);
            telemetry.update();
        }
    }

    public void calibrateGyro () throws InterruptedException {
        gyroSensor.calibrate();
        while (gyroSensor.isCalibrating()) {
            sleep(50);
            idle();
            telemetry.addData("Calibrating", "true");
            telemetry.update();
        }
        telemetry.addData("Calibrating", "false");
        telemetry.update();
    }
    public void stopRobot() {
        leftMotor.setPower(0);
        rightMotor.setPower(0);
    }

}

