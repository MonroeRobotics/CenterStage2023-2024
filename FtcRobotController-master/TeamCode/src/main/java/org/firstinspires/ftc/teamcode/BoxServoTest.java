package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

public class BoxServoTest extends OpMode {
    CRServo testServo;

    @Override
    public void init() {
        //mapping motors onto motor objects
        testServo = hardwareMap.get(CRServo.class, "testServo");
    }

    @Override
    public void loop() {
        if (gamepad1.dpad_up) testServo.setPower(1);
        else if (gamepad1.dpad_down) testServo.setPower(-1);
        else {testServo.setPower(0);}
    }



    @Override
    public void stop() {
    }
}
