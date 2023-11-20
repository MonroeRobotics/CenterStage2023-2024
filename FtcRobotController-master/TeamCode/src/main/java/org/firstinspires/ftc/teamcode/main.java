package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class main extends OpMode {

    //region variable declarations

    //region Arm Variables
    int SLIDE_HEIGHT = 20;
    double SLIDE_POWER = 0.5;
    double SLIDE_MAX_VELO = 2000;

    double ARM_POSITION = 0.05;
    double ARM_SERVO_FORWARD = 0.05;
    double ARM_SERVO_BACKWARD = 0.7;

    static double BOX_SERVO_POSITION = 1;
    static double BOX_SERVO_FORWARD = 1;
    double BOX_SERVO_BACKWARD = 0.5;


    //endregion


    //region Intake Variables
    public static double motorPower = .5;
    public static double servoPosition = .5;
    boolean intakePosition = false;
    boolean bPressed = false;
    //endregion

    //endregion

    //region Declare Objects

    //region Arm Objects
    Servo armServoRight;
    Servo armServoLeft;
    Servo boxServo;
    DcMotorEx rightLinear;
    DcMotorEx leftLinear;
    //endregion

    //region Intake Objects
    DcMotor intakeMotor;
    Servo intakeServo;
    //endregion

    //endregion



    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        //region Arm Init
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

        rightLinear.setDirection(DcMotorSimple.Direction.REVERSE);

        leftLinear.setTargetPosition(SLIDE_HEIGHT);
        rightLinear.setTargetPosition(SLIDE_HEIGHT);

        leftLinear.setPower(SLIDE_POWER);
        rightLinear.setPower(SLIDE_POWER);

        leftLinear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightLinear.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftLinear.setVelocity(SLIDE_MAX_VELO);
        rightLinear.setVelocity(SLIDE_MAX_VELO);
        //endregion

        //region Initial Servo Pos
        armServoLeft.setPosition(ARM_POSITION);
        armServoRight.setPosition(1 - ARM_POSITION);
        boxServo.setPosition(BOX_SERVO_POSITION);
        //endregion

        //endregion

        //region Intake Init
        //region Hardware Map
        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
        intakeServo = hardwareMap.get(Servo.class, "intakeServo");
        //endregion

        //region Intake Settings Settings
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeServo.setPosition(servoPosition);
        //endregion
        //endregion
    }
    @Override
    public void loop() {

        //region Arm Logic
        if (gamepad1.x) {
            BOX_SERVO_POSITION = BOX_SERVO_FORWARD;
            ARM_POSITION = ARM_SERVO_FORWARD;
        } else if (gamepad1.y) {
            BOX_SERVO_POSITION = BOX_SERVO_BACKWARD;
            ARM_POSITION = ARM_SERVO_BACKWARD;
        }

        leftLinear.setTargetPosition(SLIDE_HEIGHT);
        rightLinear.setTargetPosition(SLIDE_HEIGHT);

        armServoLeft.setPosition(ARM_POSITION);
        armServoRight.setPosition(1 - ARM_POSITION);

        boxServo.setPosition(BOX_SERVO_POSITION);
        //endregion

        //region Intake Logic
        if (gamepad1.dpad_down) {
            intakeMotor.setPower(motorPower);
        } else if (gamepad1.dpad_up) {
            intakeMotor.setPower(-motorPower);
        } else {
            intakeMotor.setPower(0);
        }

        if (gamepad1.b && bPressed == false){
            bPressed = true;
            intakePosition = !intakePosition;
            if (intakePosition == true){
                intakeServo.setPosition(0);
            }
            else{
                intakeServo.setPosition(servoPosition);
            }
        }
        else if (!gamepad1.b){
            bPressed = false;
        }
        //endregion
    }
}
