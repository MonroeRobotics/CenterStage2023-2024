package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp
public class DistanceTest extends OpMode {

    DcMotorEx motor;
    DistanceSensor distance_sensor;

    @Override
    public void init() {
        distance_sensor = hardwareMap.get(DistanceSensor.class, "distance");
        motor = hardwareMap.get(DcMotorEx.class, "motor");
        motor.setDirection(DcMotorSimple.Direction.FORWARD);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


    }


    @Override
    public void loop()
    {

        double distance = distance_sensor.getDistance(DistanceUnit.INCH);
        if (distance >= 12) {
            motor.setVelocity(600);
        }
        else {
            motor.setVelocity(0);
        }
        telemetry.addData("Distance:", distance);
        telemetry.update();
    }
}