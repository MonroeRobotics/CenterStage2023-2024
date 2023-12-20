package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class AxonTest extends OpMode {

    Servo servo1;
    Servo servo2;
    AnalogInput servo1Input;

    AnalogInput servo2Input;

    Gamepad currentGamePad;
    Gamepad previousGamePad;
    @Override
    public void init() {
        servo1 = hardwareMap.get(Servo.class, "servo1");
        servo2 = hardwareMap.get(Servo.class, "servo2");

        servo1Input = hardwareMap.get(AnalogInput.class, "servo1Input");
        servo2Input = hardwareMap.get(AnalogInput.class, "servo2Input");

        servo1.setPosition(0);
        servo2.setPosition(1);
        currentGamePad = new Gamepad();
        previousGamePad = new Gamepad();
    }

    @Override
    public void loop() {

        if(currentGamePad.cross && !previousGamePad.cross){
            servo1.setPosition(0);
            servo2.setPosition(1);
        }
        if(currentGamePad.square && !previousGamePad.square){
            servo1.setPosition(1);
            servo2.setPosition(0);
        }
        telemetry.addData("Servo 1 Reading", servo1Input.getVoltage());
        telemetry.addData("Servo 1 Reading", servo1Input.getVoltage());


    }
}
