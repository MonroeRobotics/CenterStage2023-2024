package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class ArmTuner extends OpMode {

    public static int SLIDE_HEIGHT = 20;
    public static int SLIDE_SPEED = 10;

    //region Declare Objects
    Servo armServoRight;
    Servo armServoLeft;

    Servo boxServo;
    DcMotorEx rightLinear;
    DcMotorEx leftLinear;

    //endregion


    @Override
    public void init() {
        //region Hardware Map

        armServoLeft = hardwareMap.get(Servo.class, "armServoLeft");
        armServoRight = hardwareMap.get(Servo.class, "armServoRight");
        boxServo = hardwareMap.get(Servo.class, "boxServo");
        leftLinear = hardwareMap.get(DcMotorEx.class ,"leftLinear");
        rightLinear = hardwareMap.get(DcMotorEx.class, "rightLinear");

        //endregion

        //region Motor Settings

        leftLinear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightLinear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftLinear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightLinear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftLinear.setTargetPosition(SLIDE_HEIGHT);
        rightLinear.setTargetPosition(SLIDE_HEIGHT);
        leftLinear.setVelocity(SLIDE_SPEED);
        rightLinear.setVelocity(SLIDE_SPEED);
        leftLinear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightLinear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftLinear.setDirection(DcMotorSimple.Direction.REVERSE);
        //endregion
    }
    @Override
    public void loop() {
        if(gamepad1.a){
            armServoLeft.setPosition(.2);
            armServoRight.setPosition(-.2);
        }
        else if (gamepad1.b) {
            armServoLeft.setPosition(-.2);
            armServoRight.setPosition(.2);
        }

        if (gamepad1.x) {
            boxServo.setPosition(.3);
        }
        else if (gamepad1.y){
            boxServo.setPosition(0);
        }

        leftLinear.setVelocity(SLIDE_SPEED);
        rightLinear.setVelocity(SLIDE_SPEED);
        //Reversed Due to motor direction
        leftLinear.setTargetPosition(-SLIDE_HEIGHT);
        rightLinear.setTargetPosition(SLIDE_HEIGHT);

        telemetry.addData("Slide Target Height", SLIDE_HEIGHT);
        telemetry.addData("Right Slide Height", rightLinear.getCurrentPosition());
        telemetry.addData("left Slide Height", leftLinear.getCurrentPosition());
        telemetry.update();
    }



    @Override
    public void stop() {
    }
}
