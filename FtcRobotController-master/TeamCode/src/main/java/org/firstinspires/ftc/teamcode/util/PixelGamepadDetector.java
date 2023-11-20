package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.Gamepad;
import org.firstinspires.ftc.teamcode.util.Color;


public class PixelGamepadDetector {

    Gamepad gamepad1;
    Gamepad gamepad2;

    RevColorSensorV3 colorSensor1;
    RevColorSensorV3 colorSensor2;
    Color currentColor1;
    Color previousColor1;

    Color currentColor2;
    Color previousColor2;
    Color yellow = new Color(1500, 2100, 580);

    Color green = new Color(500, 1600, 600);

    Color purple = new Color(1200, 1800, 2500);

    Color white = new Color(3000, 5000, 4500);

    public static int offset = 1000;
    Gamepad.RumbleEffect effect = new Gamepad.RumbleEffect.Builder()
            .addStep(1.0, 1.0, 500)  //  Rumble right motor 100% for 500 mSec
            .addStep(0.0, 0.0, 300)  //  Pause for 300 mSec
            .addStep(1.0, 1.0, 400)  //  Rumble left motor 100% for 250 mSec
            .addStep(0.0, 0.0, 300)  //  Pause for 250 mSec
            .addStep(1.0, 1.0, 300)  //  Rumble left motor 100% for 250 mSec
            .build();

    public PixelGamepadDetector(Gamepad gamepad1, Gamepad gamepad2, RevColorSensorV3 colorSensor1, RevColorSensorV3 colorSensor2) {

        this.gamepad1 = gamepad1;

        this.gamepad2 = gamepad2;

        this.colorSensor1 = colorSensor1;
        this.colorSensor2 = colorSensor2;

        currentColor1 = new Color(0, 0, 0);
        previousColor1 = new Color(0, 0, 0);
        currentColor2 = new Color(0, 0, 0);
        previousColor2 = new Color(0, 0, 0);
    }

    public void updateControllerColors() {
        /*if (!gamepad1.isRumbling()){
            gamepad1.runRumbleEffect(effect);
        }*/

        currentColor1.setRbg(new int[]{colorSensor1.red(), colorSensor1.green(), colorSensor1.blue()});
        currentColor2.setRbg(new int[]{colorSensor2.red(), colorSensor2.green(), colorSensor2.blue()});

        if(currentColor1.getError(previousColor1) >= 60) {
            if (!gamepad1.isRumbling()){
                gamepad1.runRumbleEffect(effect);
            }
            if (yellow.getError(currentColor1) <= offset) {
                gamepad1.setLedColor(1, 1, 0, 20000);
            } else if (purple.getError(currentColor1) <= offset) {
                gamepad1.setLedColor(1, 0, 1, 20000);
            } else if (green.getError(currentColor1) <= offset) {
                gamepad1.setLedColor(0, 1, 0, 20000);
            } else if (white.getError(currentColor1) <= offset) {
                gamepad1.setLedColor(1, 1, 1, 20000);
            } else {
                gamepad1.setLedColor(0, 0, 0, 20000);
            }
        }

        if(currentColor2.getError(previousColor2) >= 60) {
            if (yellow.getError(currentColor2) <= offset) {
                gamepad2.setLedColor(1, 1, 0, 20000);
            } else if (purple.getError(currentColor2) <= offset) {
                gamepad2.setLedColor(1, 0, 1, 20000);
            } else if (green.getError(currentColor2) <= offset) {
                gamepad2.setLedColor(0, 1, 0, 20000);
            } else if (white.getError(currentColor2) <= offset) {
                gamepad2.setLedColor(1, 1, 1, 20000);
            } else {
                gamepad2.setLedColor(0, 0, 0, 20000);
            }
        }

        previousColor1.setRbg(currentColor1.getRbg());
        previousColor2.setRbg(currentColor2.getRbg());
    }
}
