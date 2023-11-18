package org.firstinspires.ftc.teamcode;


import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.Gamepad;

@TeleOp
public class GamepadColorStuff extends OpMode {

    ColorSensor colorSensor;
    int colorDetected_argb;
    int yellow_argb;
    int purple_argb;
    int green_argb;

    int offset = 10;
    Gamepad.RumbleEffect effect = new Gamepad.RumbleEffect.Builder()
            .addStep(0.0, 1.0, 500)  //  Rumble right motor 100% for 500 mSec
            .addStep(0.0, 0.0, 300)  //  Pause for 300 mSec
            .addStep(1.0, 0.0, 250)  //  Rumble left motor 100% for 250 mSec
            .addStep(0.0, 0.0, 250)  //  Pause for 250 mSec
            .addStep(1.0, 0.0, 250)  //  Rumble left motor 100% for 250 mSec
            .build();
    @Override
    public void init() {
        colorSensor = (RevColorSensorV3) hardwareMap.get("colorSensor");
        colorSensor.enableLed(true);

    }

    @Override
    public void loop() {
        if (!gamepad1.isRumbling()){
            gamepad1.runRumbleEffect(effect);
        }
        gamepad1.setLedColor(1,1,1, 1000);

        if (Math.abs(colorDetected_argb-yellow_argb) <= offset){
            gamepad1.setLedColor(1,1,0,1000);
        }

        else if(Math.abs(colorDetected_argb-purple_argb) <= offset){
            gamepad1.setLedColor(1,0,1,1000);
        }

        else if(Math.abs(colorDetected_argb-green_argb) <= offset){
            gamepad1.setLedColor(0,1,0, 1000);
        }

        else{
            gamepad1.setLedColor(1,1,1, 1000);
        }
    }
}
