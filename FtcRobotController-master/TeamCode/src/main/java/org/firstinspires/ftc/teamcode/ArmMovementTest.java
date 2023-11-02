package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class ArmMovementTest extends OpMode {
    Servo armServoRight;
    Servo armServoLeft;

    Servo boxServo;


    @Override
    public void init() {
        //mapping motors onto motor object
        armServoLeft = hardwareMap.get(Servo.class, "armServoLeft");
        armServoRight = hardwareMap.get(Servo.class, "armServoRight");
        boxServo = hardwareMap.get(Servo.class, "boxServo");
    }
    @Override
    public void loop() {
        if(gamepad1.a){
            armServoLeft.setPosition(.2);
            armServoRight.setPosition(-.2);
        }
        else if (gamepad1.b) {
            armServoLeft.setPosition(-.2);
            armServoRight.setPosition(.2);
        }

        if (gamepad1.x) {
            boxServo.setPosition(.3);
        }
        else if (gamepad1.y){
            boxServo.setPosition(0);
        }
    }



    @Override
    public void stop() {
    }
}
