package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;

import org.firstinspires.ftc.teamcode.util.Color;

@TeleOp
@Disabled
public class ColorSensorColorFinder extends OpMode {
    RevColorSensorV3 colorSensor;
    int argbDetected;

    Color yellow = new Color(1500, 2100, 580);

    Color green = new Color(500, 1600, 600);

    Color purple = new Color(1200, 1800, 2500);

    Color white = new Color(3000, 5000, 4500);


    @Override
    public void init() {
        colorSensor = (RevColorSensorV3) hardwareMap.get("colorSensor1");}

    @Override
    public void loop() {
        argbDetected = colorSensor.argb();
        telemetry.addData("ARGB", argbDetected);
        telemetry.addData("R", colorSensor.red());
        telemetry.addData("G", colorSensor.green());
        telemetry.addData("B", colorSensor.blue());

        int error = yellow.getError(new Color(colorSensor.red(), colorSensor.green(), colorSensor.blue()));

        telemetry.addData("error", error);

        telemetry.update();
    }
}
