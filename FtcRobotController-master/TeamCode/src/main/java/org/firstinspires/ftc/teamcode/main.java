package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.util.PixelGamepadDetector;

import java.util.Timer;

@TeleOp
public class main extends OpMode {

    //region variable declarations

    //region Arm Variables

    public static int SLIDE_HEIGHT = 20;
    public static int SLIDE_STAGE = 0;
    public static double SLIDE_POWER = 0.5;
    public static double SLIDE_MAX_VELO = 2000;

    enum ArmState {
        INTAKE,
        OUTTAKE_READY,
        OUTTAKE_ACTIVE
    }

    ArmState currentArmState;

    public static double ARM_POSITION = 0.05;
    public static double ARM_SERVO_FORWARD = 0.05;
    public static double ARM_SERVO_BACKWARD = 0.7;

    public static double BOX_SERVO_POSITION = 1;
    public static double BOX_SERVO_FORWARD = 1;
    public static double BOX_SERVO_BACKWARD = 0.5;

   double outtakeTimer = 0;

    public static double OUTTAKE_TIME = 1000;


    //endregion


    //region Intake Variables
    public static double INTAKE_POWER = .5;
    public static double INTAKE_POSITION = .5;
    boolean intakeActive = false;
    boolean reverseIntake = false;

    public static double PIXEL_DETECTION_DISTANCE = .01;
    public double reverseTimer = 0;
    public static double REVERSE_TIME = 1000;


    //endregion

    //endregion

    //region Declare Objects

    SampleMecanumDrive drive;


    //region Arm Objects
    Servo armServoRight;
    Servo armServoLeft;
    Servo boxServo;
    CRServo outtakeServo;
    DcMotorEx rightLinear;
    DcMotorEx leftLinear;
    //endregion

    //region Intake Objects
    DcMotor intakeMotor;
    Servo intakeServo;

    PixelGamepadDetector pixelGamepadDetector;

    RevColorSensorV3 colorSensor1;
    RevColorSensorV3 colorSensor2;

    //endregion

    //endregion

    Gamepad currentGamepad1;
    Gamepad currentGamepad2;

    Gamepad previousGamepad1;
    Gamepad previousGamepad2;


    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        drive = new SampleMecanumDrive(hardwareMap);

        //region Arm Init
        //region Hardware Map

        armServoLeft = hardwareMap.get(Servo.class, "armServoLeft");
        armServoRight = hardwareMap.get(Servo.class, "armServoRight");
        boxServo = hardwareMap.get(Servo.class, "boxServo");
        outtakeServo = hardwareMap.get(CRServo.class,"outtakeServo");
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
        intakeServo.setPosition(INTAKE_POSITION);
        //endregion
        //endregion

        colorSensor1 = hardwareMap.get(RevColorSensorV3.class,"colorSensor1");
        colorSensor2 = hardwareMap.get(RevColorSensorV3.class, "colorSensor2");

        currentGamepad1 = new Gamepad();
        currentGamepad2 = new Gamepad();

        previousGamepad1 = new Gamepad();
        previousGamepad2 = new Gamepad();

        pixelGamepadDetector = new PixelGamepadDetector(this.gamepad1, this.gamepad2, colorSensor1, colorSensor2);
    }
    @Override
    public void loop() {

        drive.setWeightedDrivePower(new Pose2d(
                -gamepad1.left_stick_x,
                -gamepad1.left_stick_y,
                -gamepad1.right_stick_x
        ));

        //region Arm Logic


        if(currentGamepad2.dpad_up && !previousGamepad2.dpad_up && SLIDE_STAGE < 5){
            SLIDE_STAGE++;
        }
        else if(currentGamepad2.dpad_down && !previousGamepad2.dpad_down && SLIDE_STAGE > 0){
            SLIDE_STAGE--;
        }


       if(currentGamepad2.triangle && !previousGamepad2.triangle){
           switch (currentArmState){
               case INTAKE:
                   currentArmState = ArmState.OUTTAKE_READY;
                   changeArmState();
                   break;
               case OUTTAKE_READY:
                   currentArmState = ArmState.OUTTAKE_ACTIVE;
                   outtakeTimer = System.currentTimeMillis() + OUTTAKE_TIME;
                   changeArmState();
                   break;
           }
       }
       else if(currentGamepad2.square && !previousGamepad2.square){
           currentArmState = ArmState.INTAKE;
           changeArmState();
       }

       if(currentArmState == ArmState.OUTTAKE_ACTIVE && outtakeTimer < System.currentTimeMillis()){
           currentArmState = ArmState.INTAKE;
           changeArmState();
       }



        leftLinear.setTargetPosition(SLIDE_HEIGHT);
        rightLinear.setTargetPosition(SLIDE_HEIGHT);

        armServoLeft.setPosition(ARM_POSITION);
        armServoRight.setPosition(1 - ARM_POSITION);

        boxServo.setPosition(BOX_SERVO_POSITION);
        //endregion

        //region Intake Logic
        if (currentGamepad1.cross && !previousGamepad1.cross){
            intakeActive = true;
        }

        if (currentGamepad1.circle && !previousGamepad1.circle){
            intakeActive = false;
            reverseIntake = false;
        }

        if(intakeActive){

            //
            intakeMotor.setPower(INTAKE_POWER);
            intakeServo.setPosition(INTAKE_POSITION);

            //Runs outtake servo
            outtakeServo.setPower(-1);

            if(colorSensor1.getDistance(DistanceUnit.CM) <= PIXEL_DETECTION_DISTANCE && colorSensor2.getDistance(DistanceUnit.CM) <= PIXEL_DETECTION_DISTANCE){
                intakeActive = false;
                reverseIntake = true;
                reverseTimer = System.currentTimeMillis() + REVERSE_TIME;
            }
        }
        else if (reverseIntake && reverseTimer > System.currentTimeMillis()){
            intakeMotor.setPower(-INTAKE_POWER);
        }
        else{
            intakeMotor.setPower(INTAKE_POWER);
            intakeServo.setPosition(0);
        }
        //endregion

        pixelGamepadDetector.updateControllerColors();

        previousGamepad1.copy(currentGamepad1);
        previousGamepad2.copy(currentGamepad2);

        currentGamepad1.copy(gamepad1);
        currentGamepad2.copy(gamepad2);

    }

    public void changeArmState(){
        switch (currentArmState){
            case INTAKE:
                BOX_SERVO_POSITION = BOX_SERVO_FORWARD;
                ARM_POSITION = ARM_SERVO_FORWARD;
                SLIDE_HEIGHT = 20;
                outtakeServo.setPower(1);
                break;
            case OUTTAKE_READY:
                BOX_SERVO_POSITION = BOX_SERVO_BACKWARD;
                ARM_POSITION = ARM_SERVO_BACKWARD;
                if (SLIDE_STAGE == 0) {
                    SLIDE_HEIGHT = 540;
                }
                else{
                    SLIDE_HEIGHT = 540 + (SLIDE_STAGE * 150);
                }
                break;
            case OUTTAKE_ACTIVE:
                outtakeServo.setPower(1);
        }
    }
}
