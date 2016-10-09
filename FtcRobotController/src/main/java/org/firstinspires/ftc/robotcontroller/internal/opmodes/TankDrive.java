package org.firstinspires.ftc.robotcontroller.internal.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by Administrator on 9/25/2016.
 */
public class TankDrive extends OpMode {

    DcMotor leftMotor;
    DcMotor rightMotor;

    @Override
    public void init() {
        leftMotor = hardwareMap.dcMotor.get("left_drive");
        rightMotor = hardwareMap.dcMotor.get("right_drive");

//        rightMotor.setDirection(DcMotor.Direction.REVERSE);
    }

    @Override
    public void loop() {
        float leftY = -gamepad1.left_stick_y;
        float rightY = -gamepad1.right_stick_y;
//        leftY = (leftY == 0) ? -gamepad2.left_stick_y : leftY;
//        rightY = (rightY == 0) ? -gamepad2.right_stick_y : rightY;

        telemetry.addData("left joystick value", leftY);
        telemetry.addData("right joystick value", rightY);

        leftMotor.setPower(leftY);
        rightMotor.setPower(rightY);
    }
}

