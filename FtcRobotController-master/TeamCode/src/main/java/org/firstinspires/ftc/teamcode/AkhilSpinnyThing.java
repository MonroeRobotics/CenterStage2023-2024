package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

public class AkhilSpinnyThing extends OpMode {
    DcMotor spinny;
    Servo uppy;
    double motorPower = .5;
    double servoPosition = .3;
    boolean intakePosition = false;
    boolean pressed = false;
//use dpad for intake and outtake
//1 motor for powering and one servo for up and down
    @Override
    public void init() {
        //mapping motors onto motor objects
        spinny = hardwareMap.get(DcMotor.class, "spinny");
        uppy = hardwareMap.get(Servo.class, "uppy");
        //setting motors runmode
        spinny.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    @Override
    public void loop() {
        if (gamepad1.dpad_down) {
           spinny.setPower(motorPower);
        }
        else {
            //power zero
        }
        if (gamepad1.x){
            if (pressed == false){
                pressed = true;
                intakePosition = !intakePosition;
                if (intakePosition == true){
                    uppy.setPosition(servoPosition);
                }
                else{
                    uppy.setPosition(0);
                }
            }

        }
        else{
            pressed = false;
        }
    }



    @Override
    public void stop() {
        motorPower = 0;
        spinny.setPower(motorPower);
    }
}
