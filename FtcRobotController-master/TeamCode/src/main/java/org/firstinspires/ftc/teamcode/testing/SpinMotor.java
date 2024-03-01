package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;

@TeleOp
public class SpinMotor extends OpMode {

    DcMotor motor;
    Gamepad currentGamepad1 = new Gamepad();
    int targetPos = 1;
    public void init() {
        motor = hardwareMap.get(DcMotor.class, "motor");
        motor.setTargetPosition(targetPos);
        targetPos = 50;
    }
    public void loop() {
        if (Math.abs(gamepad1.left_stick_y) >= 0.1) motor.setPower(gamepad1.left_stick_y);
        else motor.setPower(0);

        currentGamepad1.copy(gamepad1);
    }
}