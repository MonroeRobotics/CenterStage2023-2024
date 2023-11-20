package org.firstinspires.ftc.teamcode;


import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.Gamepad;

public class PixelGamepadDetector {

    ColorSensor colorSensor1;
    ColorSensor colorSensor2;
    int colorDetected_argb;
    int yellow_argb;
    int purple_argb;
    int green_argb;
    int white_argb;

    int offset = 10;

    Gamepad gamepad1;
    Gamepad gamepad2;

    public PixelGamepadDetector(ColorSensor colorSensor1, ColorSensor colorSensor2, Gamepad gamepad1, Gamepad gamepad2) {
        this.colorSensor1 = colorSensor1;
        this.colorSensor2 = colorSensor2;

        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;
    }

    Gamepad.RumbleEffect effect = new Gamepad.RumbleEffect.Builder()
            .addStep(0.0, 1.0, 500)  //  Rumble right motor 100% for 500 mSec
            .addStep(0.0, 0.0, 300)  //  Pause for 300 mSec
            .addStep(1.0, 0.0, 250)  //  Rumble left motor 100% for 250 mSec
            .addStep(0.0, 0.0, 250)  //  Pause for 250 mSec
            .addStep(1.0, 0.0, 250)  //  Rumble left motor 100% for 250 mSec
            .build();



    public void updateController() {
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

        else if(Math.abs(colorDetected_argb-white_argb) <= offset){
            gamepad1.setLedColor(1,1,1, 1000);
        }
        else{
            gamepad1.setLedColor(0,0,0,1000);
        }
    }
}
