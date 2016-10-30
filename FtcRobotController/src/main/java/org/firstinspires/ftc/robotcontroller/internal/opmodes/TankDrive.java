package org.firstinspires.ftc.robotcontroller.internal.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by Administrator on 9/25/2016.
 */
public class TankDrive extends OpMode {

    DcMotor leftMotor;
    DcMotor rightMotor;
    DcMotor conveyor;
    DcMotor shooter;

    @Override
    public void init() {
        leftMotor = hardwareMap.dcMotor.get("left_drive");
        rightMotor = hardwareMap.dcMotor.get("right_drive");
        conveyor = hardwareMap.dcMotor.get("conveyor");
        shooter = hardwareMap.dcMotor.get("shooter");

//      rightMotor.setDirection(DcMotor.Direction.REVERSE);
    }

    @Override
    public void loop() {
        float leftY = -gamepad1.left_stick_y;
        float rightY = gamepad1.right_stick_y;
        float rightTrigger = gamepad2.right_trigger;
        float leftTrigger = -gamepad2.left_trigger;
        int hi = 0;
        leftMotor.setPower(leftY);
        rightMotor.setPower(rightY);
//        leftY = (leftY == 0) ? -gamepad2.left_stick_y : leftY;
//        rightY = (rightY == 0) ? -gamepad2.right_stick_y : rightY;
        if (rightTrigger > 0) {
            conveyor.setPower(rightTrigger);
        } else if (leftTrigger < 0) {
            conveyor.setPower(leftTrigger);
        } else {
            conveyor.setPower(0);
        }
//If gamepad 2's A button is pressed, set shooter's motor to one
        if (gamepad2.a) {
            shooter.setPower(1);
        } else {
            shooter.setPower(0);
        }


        telemetry.addData("left joystick value", leftY);
        telemetry.addData("right joystick value", rightY);
        telemetry.addData("Right trigger value", rightTrigger);
        telemetry.addData("Left trigger value", leftTrigger);


    }
}

