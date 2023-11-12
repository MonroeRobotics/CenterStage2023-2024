package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@TeleOp
public class AkhilSpinnyThing extends OpMode {
    DcMotor spinny;
    Servo uppy;
    public static double motorPower = .5;
    public static double servoPosition = .5;
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
        spinny.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        uppy.setPosition(servoPosition);
    }

    @Override
    public void loop() {
        if (gamepad1.dpad_down) {
           spinny.setPower(motorPower);
        } else if (gamepad1.dpad_up) {
            spinny.setPower(-motorPower);
        } else {
            spinny.setPower(0);
        }

        if (gamepad1.x && pressed == false){
            pressed = true;
            intakePosition = !intakePosition;
            if (intakePosition == true){
                uppy.setPosition(0);
            }
            else{
                uppy.setPosition(servoPosition);
            }
        }
        else if (!gamepad1.x){
            pressed = false;
        }
    }



    @Override
    public void stop() {
        motorPower = 0;
        spinny.setPower(motorPower);
    }
}
