package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp
public class ColorSensorTest extends OpMode {

    DcMotorEx motor;
    RevColorSensorV3 colorSensorV3;

    @Override
    public void init() {
        colorSensorV3 = (RevColorSensorV3) hardwareMap.colorSensor.get("colorV3");
        motor = hardwareMap.get(DcMotorEx.class, "motor");
        motor.setDirection(DcMotorSimple.Direction.FORWARD);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }


    @Override
    public void loop() {
        colorSensorV3.enableLed(true);

        if(colorSensorV3.red() >= 200){
            motor.setVelocity(1000);
        }else{
            motor.setVelocity(0);
        }

    }
}