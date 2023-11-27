package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public class suspensionTuner extends OpMode{

    DcMotorEx hangMotor;
    @Override
    public void init() {
        hangMotor = hardwareMap.get(DcMotorEx.class, "hangMotor");
    }

    @Override
    public void loop() {
        if(gamepad1.dpad_up) {
            hangMotor.setPower(1);
        } else if (gamepad1.dpad_down) {
            hangMotor.setPower(-1);
        }
        else {
            hangMotor.setPower(0);
        }
    }
}
