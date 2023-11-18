package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;

public class ColorSensorColorFinder extends OpMode {
    RevColorSensorV3 colorSensor;
    int argbDetected;

    @Override
    public void init() {
        colorSensor = (RevColorSensorV3) hardwareMap.get("colorSensor");
    }

    @Override
    public void loop() {
        argbDetected = colorSensor.argb();
        telemetry.addData("ARGB", argbDetected);
        telemetry.update();
    }
}
