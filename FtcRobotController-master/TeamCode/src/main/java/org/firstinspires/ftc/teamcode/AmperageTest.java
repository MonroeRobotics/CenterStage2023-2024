package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@TeleOp
public class AmperageTest extends OpMode {

    DcMotorEx motor;
    double currentThreshold = 0.3;

    boolean reverse = false;

    double timer = 0;

    @Override
    public void init() {
        motor = hardwareMap.get(DcMotorEx.class, "motor");
        motor.setDirection(DcMotorSimple.Direction.FORWARD);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }


    @Override
    public void loop() {

        if(motor.getCurrent(CurrentUnit.AMPS)>currentThreshold){
            reverse = true;
            timer = System.currentTimeMillis() + 500;
        }
        else if (timer > System.currentTimeMillis()){
            reverse = false;
        }

        if(reverse){
            motor.setPower(-0.1);
        }
        else {
            motor.setPower(0.05);
        }
        telemetry.addData("Current Draw Motor:", motor.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("Current Motor Power:", motor.getPower());
        telemetry.addData("Timer", timer);

    }
}