package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
@Config
public class ArmTuner extends OpMode {

    public static int SLIDE_HEIGHT = 20;
    public static double SLIDE_POWER = 0.5;

    public static double ARM_SERVO_POSITION = 0;
    public static double ARM_SERVO_FORWARD_MAX = 0.1;
    public static double ARM_SERVO_BACKWARD_MAX = -0.1;

    public static double BOX_SERVO_POSITION = 0;
    public static double BOX_SERVO_FORWARD_MAX = 0.1;
    public static double BOX_SERVO_BACKWARD_MAX = -0.1;

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
        leftLinear.setPower(SLIDE_POWER);
        rightLinear.setPower(SLIDE_POWER);
        leftLinear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightLinear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //endregion

        //region Initial Servo Pos
        armServoLeft.setPosition(ARM_SERVO_POSITION);
        armServoRight.setPosition(ARM_SERVO_POSITION);
        boxServo.setPosition(BOX_SERVO_POSITION);
        //endregion
    }
    @Override
    public void loop() {
        if(gamepad1.a){

            ARM_SERVO_POSITION = ARM_SERVO_FORWARD_MAX;
        }
        else if (gamepad1.b) {
            ARM_SERVO_POSITION = ARM_SERVO_BACKWARD_MAX;
        }

        if (gamepad1.x) {
            BOX_SERVO_POSITION = BOX_SERVO_FORWARD_MAX;
        }
        else if (gamepad1.y){
            BOX_SERVO_POSITION = BOX_SERVO_BACKWARD_MAX;
        }

        leftLinear.setTargetPosition(SLIDE_HEIGHT);
        rightLinear.setTargetPosition(SLIDE_HEIGHT);

        armServoLeft.setPosition(ARM_SERVO_POSITION);
        armServoRight.setPosition(ARM_SERVO_POSITION);

        boxServo.setPosition(BOX_SERVO_POSITION);

        telemetry.addData("Slide Target Height", SLIDE_HEIGHT);
        telemetry.addData("Right Slide Height", rightLinear.getCurrentPosition());
        telemetry.addData("left Slide Height", leftLinear.getCurrentPosition());
        telemetry.update();
    }



    @Override
    public void stop() {
    }
}
