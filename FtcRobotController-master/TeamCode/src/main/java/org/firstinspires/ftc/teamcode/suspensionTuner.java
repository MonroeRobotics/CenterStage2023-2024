package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp
public class suspensionTuner extends OpMode{

    DcMotorEx hangMotor;

    public static int riggingHeight = 0;
    @Override
    public void init() {
        hangMotor = hardwareMap.get(DcMotorEx.class, "hangMotor");
        hangMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hangMotor.setPower(1);
        hangMotor.setTargetPosition(riggingHeight);

        hangMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

    }

    @Override
    public void loop() {
        if(gamepad1.dpad_up && riggingHeight <= 1000) {
            riggingHeight += 5;
        } else if (gamepad1.dpad_down && riggingHeight >= 0) {
            riggingHeight -= 5;
        }


        hangMotor.setTargetPosition(riggingHeight);
    }
}
