package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
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

import org.firstinspires.ftc.robotcore.external.Const;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.util.PixelGamepadDetector;

import java.util.Timer;

@TeleOp
@Config
public class main extends OpMode {

    //region variable declarations

    //region Arm Variables

    public static int SLIDE_HEIGHT = 20; //Live Updating Slide height
    public static int SLIDE_STAGE = 0; //Used for incremental Slide Height
    public static double SLIDE_POWER = 0.5; //Max Linear Slide Power
    public static double SLIDE_MAX_VELO = 2000; //Max Linear Slide Velocity


    enum ArmState { //Creates States that arm could be in for logic use
        INTAKE,
        OUTTAKE_READY,
        OUTTAKE_ACTIVE
    }

    ArmState currentArmState = ArmState.INTAKE; //Creates a variables to store current Arm State

    public static double ARM_POSITION = 0.04; //Live Updating Arm Servo Position (1 is intake position)
    public static double ARM_SERVO_FORWARD = 0.04;//Stores Value of Arm intake Position
    public static double ARM_SERVO_BACKWARD = 0.7;//Stores Value of Arm outtake Position

    public static double BOX_SERVO_POSITION = 1; //Live Updating Box Position (1 is intake position)
    public static double BOX_SERVO_FORWARD = 1; //Stores Value of Box intake Position
    public static double BOX_SERVO_BACKWARD = 0.3; //Stores value of Box Outtake position

    double outtakeTimer = 0; //Timer to control outtake
    public static double OUTTAKE_TIME = 1000; //How Long Outtake runs for
    //endregion


    //region Intake Variables
    public static double INTAKE_POWER = .5; //Power of Intake Motor
    public static double INTAKE_POSITION = .3; //Position of Intake Servo
    boolean intakeActive = false;
    boolean reverseIntake = false;

    public static double PIXEL_DETECTION_DISTANCE = 1; //Distance from color sensor to pixel for detection (CM)
    public double reverseTimer = 0; //timer for reversing intake
    public static double REVERSE_TIME = 1000; //How long to Reverse intake


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


    //Creates Gamepad objects to store current and previous Gamepad states
    Gamepad currentGamepad1;
    Gamepad currentGamepad2;

    Gamepad previousGamepad1;
    Gamepad previousGamepad2;


    @Override
    public void init() {
        //Sets up telemetry to work with FTC Dashboard
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        //creates and sets up RR Mecanum Drive
        drive = new SampleMecanumDrive(hardwareMap);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //region Arm Init
        //region Arm Hardware Map

        armServoLeft = hardwareMap.get(Servo.class, "armServoLeft");
        armServoRight = hardwareMap.get(Servo.class, "armServoRight");
        boxServo = hardwareMap.get(Servo.class, "boxServo");
        outtakeServo = hardwareMap.get(CRServo.class,"outtakeServo");
        leftLinear = hardwareMap.get(DcMotorEx.class ,"leftLinear");
        rightLinear = hardwareMap.get(DcMotorEx.class, "rightLinear");

        //endregion

        //region Arm Lift Motor Settings
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
        //region Intake Hardware Map
        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
        intakeServo = hardwareMap.get(Servo.class, "intakeServo");
        //endregion

        //region Intake Settings
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeServo.setPosition(1);
        //endregion
        //endregion

        colorSensor1 = hardwareMap.get(RevColorSensorV3.class,"colorSensor1");
        colorSensor2 = hardwareMap.get(RevColorSensorV3.class, "colorSensor2");

        currentGamepad1 = new Gamepad();
        currentGamepad2 = new Gamepad();

        previousGamepad1 = new Gamepad();
        previousGamepad2 = new Gamepad();

        //See util.PixelGamepadDetector
        pixelGamepadDetector = new PixelGamepadDetector(this.gamepad1, this.gamepad2, colorSensor1, colorSensor2);
    }
    @Override
    public void loop() {

        drive.setWeightedDrivePower(new Pose2d(
                -gamepad1.left_stick_y,
                -gamepad1.left_stick_x,
                -gamepad1.right_stick_x
        ));

        //region Arm Logic

        //Increases or Decreases Slide Stage on Dpad up or down within [0,7]
        if(currentGamepad2.dpad_up && !previousGamepad2.dpad_up && SLIDE_STAGE < 7){
            SLIDE_STAGE++;
            changeArmState();
        }
        else if(currentGamepad2.dpad_down && !previousGamepad2.dpad_down && SLIDE_STAGE > 0){
            SLIDE_STAGE--;
            changeArmState();
        }

        //Changes Arm State on Triangle Press
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
       //Resets Arm to Intake Position on square press
       else if(currentGamepad2.square && !previousGamepad2.square){
           currentArmState = ArmState.INTAKE;
           changeArmState();
       }

       //Changes Arm State back to intake once outtake timer runs out
       if(currentArmState == ArmState.OUTTAKE_ACTIVE && outtakeTimer < System.currentTimeMillis()){
           currentArmState = ArmState.INTAKE;
           changeArmState();
       }


        //Sets Slides Arm and Box to respective positions as determined by the previous logic
        leftLinear.setTargetPosition(SLIDE_HEIGHT);
        rightLinear.setTargetPosition(SLIDE_HEIGHT);

        armServoLeft.setPosition(ARM_POSITION);
        armServoRight.setPosition(1 - ARM_POSITION);

        boxServo.setPosition(BOX_SERVO_POSITION);
        //endregion

        //region Intake Logic

        //On cross start intake (Only if arm is in intake position)
        if(currentArmState == ArmState.INTAKE){
            if (currentGamepad2.cross && !previousGamepad2.cross){
                intakeActive = true;
            }
            else if (currentGamepad2.circle && !previousGamepad2.circle){
                intakeActive = false;
                reverseIntake = false;
                outtakeServo.setPower(0);
            }
        }
        //if arm is not in take position cancel intake
        else {
            intakeActive = false;
        }
        //On right bumper reverse intake
        if (currentGamepad2.right_bumper && !previousGamepad2.right_bumper) {
            intakeActive = false;
            reverseTimer = System.currentTimeMillis() + REVERSE_TIME;
            reverseIntake = true;
            outtakeServo.setPower(0);
        }

        if(intakeActive){

            intakeMotor.setPower(INTAKE_POWER);
            intakeServo.setPosition(INTAKE_POSITION);

            outtakeServo.setPower(-1);

            //If two pixels are detected within specified distance, start reversal of intake
            if(colorSensor1.getDistance(DistanceUnit.CM) <= PIXEL_DETECTION_DISTANCE && colorSensor2.getDistance(DistanceUnit.CM) <= PIXEL_DETECTION_DISTANCE){
                intakeActive = false;
                reverseIntake = true;
                reverseTimer = System.currentTimeMillis() + REVERSE_TIME;
            }
        }
        //Checks if reverse is on and timer is still on
        else if (reverseIntake && reverseTimer > System.currentTimeMillis()){
            intakeServo.setPosition(INTAKE_POSITION);
            intakeMotor.setPower(-INTAKE_POWER);
            outtakeServo.setPower(0);
        }
        else{
            intakeMotor.setPower(0);
            intakeServo.setPosition(1);
        }
        //endregion

        drive.update();

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
                outtakeServo.setPower(0);
                SLIDE_HEIGHT = 20;
                break;
            case OUTTAKE_READY:
                BOX_SERVO_POSITION = BOX_SERVO_BACKWARD;
                ARM_POSITION = ARM_SERVO_BACKWARD;
                outtakeServo.setPower(0);
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
