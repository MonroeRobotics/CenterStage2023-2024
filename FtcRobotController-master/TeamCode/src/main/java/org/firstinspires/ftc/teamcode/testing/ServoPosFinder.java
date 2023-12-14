package org.firstinspires.ftc.teamcode.testing;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
@Config
public class ServoPosFinder extends OpMode {
    Servo servo;
    public static double servoPos = 0;
    @Override
    public void init() {
        servo = hardwareMap.get(Servo.class, "droneServo");

    }

    @Override
    public void loop() {

        servo.setPosition(servoPos);
        telemetry.addData("Servo Pos", servoPos);
        telemetry.update();
    }
}
