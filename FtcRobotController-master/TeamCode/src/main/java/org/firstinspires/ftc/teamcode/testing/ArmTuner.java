package org.firstinspires.ftc.teamcode.testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@TeleOp
@Config
public class ArmTuner extends OpMode {

    public static int SLIDE_HEIGHT = 20;
    public static double SLIDE_POWER = 0.5;
    public static double SLIDE_MAX_VELO = 2000;

    public static double ARM_POSITION = 0.05;
    public static double ARM_SERVO_FORWARD = 0.05;
    public static double ARM_SERVO_BACKWARD = 0.7;

    public static double BOX_SERVO_POSITION = 1;
    public static double BOX_SERVO_FORWARD = 1;
    public static double BOX_SERVO_BACKWARD = 0.5;

    /*public static double Pv = 1;
    public static double Iv = 0;
    public static double Dv = 0;
    public static double Fv = 10;
    public static double Pp = 1;
*/




    //region Declare Objects
    Servo armServoRight;
    Servo armServoLeft;

    Servo boxServo;
    DcMotorEx rightLinear;
    DcMotorEx leftLinear;

    //endregion


    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

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

        /*leftLinear.setPositionPIDFCoefficients(Pp);
        rightLinear.setPositionPIDFCoefficients(Pp);

        leftLinear.setVelocityPIDFCoefficients(Pv, Iv, Dv, Fv);
        rightLinear.setVelocityPIDFCoefficients(Pv, Iv, Dv, Fv);*/

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
    }
    @Override
    public void loop() {
        if (gamepad1.x) {
            BOX_SERVO_POSITION = BOX_SERVO_FORWARD;
            ARM_POSITION = ARM_SERVO_FORWARD;
        }
        else if (gamepad1.y){
            BOX_SERVO_POSITION = BOX_SERVO_BACKWARD;
            ARM_POSITION = ARM_SERVO_BACKWARD;
        }

        leftLinear.setTargetPosition(SLIDE_HEIGHT);
        rightLinear.setTargetPosition(SLIDE_HEIGHT);

        armServoLeft.setPosition(ARM_POSITION);
        armServoRight.setPosition(1 - ARM_POSITION);

        boxServo.setPosition(BOX_SERVO_POSITION);



        telemetry.addData("Slide Target Height", SLIDE_HEIGHT);
        telemetry.addData("Right Slide Height", rightLinear.getCurrentPosition());
        telemetry.addData("left Slide Height", leftLinear.getCurrentPosition());
        telemetry.addData("Right Slide Volt", rightLinear.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("left Slide Volt", leftLinear.getCurrent(CurrentUnit.AMPS));


        telemetry.addData("Right Servo Pos:", armServoRight.getPosition());
        telemetry.addData("Left Servo Pos:", armServoLeft.getPosition());

        telemetry.update();
    }



    @Override
    public void stop() {
    }
}
