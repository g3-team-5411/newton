package org.firstinspires.ftc.robotcontroller.internal.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import static android.os.SystemClock.sleep;

/**
 * Created by Administrator on 10/16/2016.
 */

public class GreenPractice extends OpMode {

    private DcMotor leftMotor;
    private DcMotor rightMotor;


    @Override
    public void init() {
        leftMotor = hardwareMap.dcMotor.get("left_drive");
        rightMotor = hardwareMap.dcMotor.get("right_drive");
        rightMotor.setDirection(DcMotor.Direction.REVERSE);
    }

    public void loop() {}
    public void start() {
        //driveForward(){
        leftMotor.setPower(0.5);
        rightMotor.setPower(0.5);
        sleep(2000);

        //stop robot
        leftMotor.setPower(0.0);
        rightMotor.setPower(0.0);
        sleep(2000);

        //turn robot(){
        leftMotor.setPower(0.5);
        rightMotor.setPower(0.0);
        sleep(2000);

        //driveForward(){
        leftMotor.setPower(0.5);
        rightMotor.setPower(0.5);
        sleep(2000);

        //stop robot
        leftMotor.setPower(0.0);
        rightMotor.setPower(0.0);
        sleep(2000);


    }
}