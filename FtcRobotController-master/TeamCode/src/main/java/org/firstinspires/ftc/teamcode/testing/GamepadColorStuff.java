package org.firstinspires.ftc.teamcode.testing;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.util.Color;

@Config
@TeleOp
@Disabled
public class GamepadColorStuff extends OpMode {

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
            .addStep(0.0, 1.0, 500)  //  Rumble right motor 100% for 500 mSec
            .addStep(0.0, 0.0, 300)  //  Pause for 300 mSec
            .addStep(1.0, 0.0, 250)  //  Rumble left motor 100% for 250 mSec
            .addStep(0.0, 0.0, 250)  //  Pause for 250 mSec
            .addStep(1.0, 0.0, 250)  //  Rumble left motor 100% for 250 mSec
            .build();
    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        colorSensor1 = (RevColorSensorV3) hardwareMap.get("colorSensor1");
        colorSensor2 = (RevColorSensorV3) hardwareMap.get("colorSensor2");
        colorSensor1.enableLed(true);
        colorSensor2.enableLed(true);

        currentColor1 = new Color(0, 0, 0);
        previousColor1 = new Color(0, 0, 0);
        currentColor2 = new Color(0, 0, 0);
        previousColor2 = new Color(0, 0, 0);
    }

    @Override
    public void loop() {
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

        telemetry.addData("Change In C", currentColor1.getError(previousColor1));
        previousColor1.setRbg(currentColor1.getRbg());
        previousColor2.setRbg(currentColor2.getRbg());

        telemetry.addData("W Error", white.getError(currentColor1));
        telemetry.addData("Y Error", yellow.getError(currentColor1));
        telemetry.addData("P Error", purple.getError(currentColor1));
        telemetry.addData("G Error", green.getError(currentColor1));

        telemetry.addData("2 W Error", white.getError(currentColor2));
        telemetry.addData("2 Y Error", yellow.getError(currentColor2));
        telemetry.addData("2 P Error", purple.getError(currentColor2));
        telemetry.addData("2 G Error", green.getError(currentColor2));

        telemetry.update();
    }
}
