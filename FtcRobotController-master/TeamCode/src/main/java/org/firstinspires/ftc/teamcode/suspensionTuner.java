package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
@Config
public class suspensionTuner extends OpMode{

    DcMotorEx hangMotor;
    Servo servo;
    public static double servoPos = 0;

    public static int riggingHeight = 0;
    public static int riggingHeightMin = 0;

    public static int riggingHeightMax = 14500; //

    //drone rigging height max 5620

    public static int droneStart = 0;
    public static double droneRelease = 0.3;

    Gamepad currentGamepad;

    Gamepad prevoiousGamepad;

    @Override
    public void init() {
        hangMotor = hardwareMap.get(DcMotorEx.class, "hangMotor");
        servo = hardwareMap.get(Servo.class, "droneServo");
        hangMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        hangMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        hangMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        hangMotor.setPower(1);
        hangMotor.setTargetPosition(riggingHeight);

        hangMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        servo.setPosition(servoPos);

        currentGamepad = new Gamepad();
        prevoiousGamepad = new Gamepad();
        currentGamepad.copy(gamepad1);
        prevoiousGamepad.copy(gamepad1);
    }

    @Override
    public void loop() {
        if (gamepad1.right_bumper && gamepad1.left_bumper && gamepad1.dpad_up){
            riggingHeight += 5;
        } else if (gamepad1.right_bumper && gamepad1.left_bumper && gamepad1.dpad_down) {
            riggingHeight -= 5;
        }
        else if(gamepad1.dpad_up && riggingHeight <= riggingHeightMax) {
            riggingHeight += 5;
        } else if (gamepad1.dpad_down && riggingHeight >= riggingHeightMin) {
            riggingHeight -= 5;
        }
        hangMotor.setTargetPosition(riggingHeight);

        telemetry.addData("Rigging Height", riggingHeight);
        telemetry.update();

        if(currentGamepad.a && !prevoiousGamepad.a){
            if(servoPos == droneStart) {
                servoPos = droneRelease;
            }
            else {
                servoPos = droneStart;
            }
        }

        servo.setPosition(servoPos);

        prevoiousGamepad.copy(currentGamepad);
        currentGamepad.copy(gamepad1);
    }
}
