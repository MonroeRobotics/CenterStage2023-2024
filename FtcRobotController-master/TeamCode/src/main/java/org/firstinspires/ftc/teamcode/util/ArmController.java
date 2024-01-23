package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;


@Config
public class ArmController {
    HardwareMap hardwareMap;

    //region Arm Variables

    int SLIDE_HEIGHT = 20; //Live Updating Slide height
    int SLIDE_STAGE = 0; //Used for incremental Slide Height
    public static double SLIDE_POWER = 0.5; //Max Linear Slide Power
    public static double SLIDE_MAX_VELO = 2000; //Max Linear Slide Velocity


    public enum ArmState { //Creates States that arm could be in for logic use
        INTAKE,
        OUTTAKE_READY,
        OUTTAKE_ACTIVE
    }

    ArmState currentArmState = ArmState.INTAKE; //Creates a variables to store current Arm State

    double ARM_POSITION = 0; //Live Updating Arm Servo Position (0 is intake position)
    public static double ARM_SERVO_FORWARD = 0;//Stores Value of Arm intake Position
    public static double ARM_SERVO_BACKWARD = 0.9;//Stores Value of Arm outtake Position

    double BOX_SERVO_POSITION = .2; //Live Updating Box Position (.15 is intake position)
    public static double BOX_SERVO_FORWARD = .2; //Stores Value of Box intake Position
    public static double BOX_SERVO_TRANSITION = 0.6; //Stores value of Box Outtake position
    public static double BOX_SERVO_BACKWARD = 0.85; //Stores value of Box Outtake position
    public static int SLIDE_HEIGHT_SERVO_TRANSITION = 1000;

    double outtakeTimer = 0; //Timer to control outtake
    public static double OUTTAKE_TIME = 150; //How Long Outtake runs for (ms)
    //endregion

    //region Arm Objects
    Servo armServoRight;
    Servo armServoLeft;
    Servo boxServo;
    CRServo outtakeServo;
    DcMotorEx rightLinear;
    DcMotorEx leftLinear;
    //endregion

    public ArmController (HardwareMap hardwareMap){
        this.hardwareMap = hardwareMap;
    }

    public void initArm(){
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
    }
    public void switchArmState(){
        switch (currentArmState) {
            case INTAKE:
                currentArmState = ArmState.OUTTAKE_READY;
                updateArmState();
                break;
            case OUTTAKE_READY:
                currentArmState = ArmState.INTAKE;
                updateArmState();
                break;
        }
    }

    public void setOuttakePower(double power){
        outtakeServo.setPower(power);
    }


    public void updateArmState(){
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
                    SLIDE_HEIGHT = 450;
                }
                else{
                    SLIDE_HEIGHT = 450 + (SLIDE_STAGE * 150);
                }
                break;
        }
    }

    public void changeStage(int change){
        if (change > 0 && SLIDE_STAGE < 7) {
            SLIDE_STAGE += change;
        } else if (change < 0 && SLIDE_STAGE > 0) {
            SLIDE_STAGE += change;
        }
        updateArmState();
    }

    public int getSlideHeight(){
        return SLIDE_HEIGHT;
    }


    public void setSlideHeight(int slideHeight){
        SLIDE_HEIGHT = slideHeight;
    }

    public ArmState getCurrentArmState(){
        return currentArmState;
    }


    public void setArmPos(double armPos){
        ARM_POSITION = armPos;
    }

    public void setBoxPos(double boxPos){
        BOX_SERVO_POSITION = boxPos;

    }

    public void startOuttake(){
        if (outtakeTimer <= System.currentTimeMillis()) {
            outtakeTimer = System.currentTimeMillis() + OUTTAKE_TIME;
            outtakeServo.setPower(1);
        }
        else if (outtakeTimer >= System.currentTimeMillis() && outtakeTimer <= (System.currentTimeMillis() + (OUTTAKE_TIME * 2))){
            outtakeTimer += OUTTAKE_TIME;
            outtakeServo.setPower(1);
        }
    }

    public void checkOuttakeTimer(){
        if(outtakeTimer <= System.currentTimeMillis() && currentArmState == ArmState.OUTTAKE_READY){
            outtakeServo.setPower(0);
        }
    }


    public void updateArm(){
        //Sets Slides Arm and Box to respective positions as determined by the previous logic
        leftLinear.setTargetPosition(SLIDE_HEIGHT);
        rightLinear.setTargetPosition(SLIDE_HEIGHT);

        armServoLeft.setPosition(ARM_POSITION);
        armServoRight.setPosition(1 - ARM_POSITION);

        if (currentArmState == ArmState.INTAKE && leftLinear.getCurrentPosition() <= SLIDE_HEIGHT_SERVO_TRANSITION){
            boxServo.setPosition(BOX_SERVO_POSITION);
        }
        else if(currentArmState == ArmState.INTAKE) {
            boxServo.setPosition(BOX_SERVO_TRANSITION);
        }
        else if(currentArmState != ArmState.INTAKE) {
            boxServo.setPosition(BOX_SERVO_POSITION);
        }
        checkOuttakeTimer();
    }

    public void updateArmABS(){
        //Sets Slides Arm and Box to respective positions as determined by the previous logic
        leftLinear.setTargetPosition(SLIDE_HEIGHT);
        rightLinear.setTargetPosition(SLIDE_HEIGHT);

        armServoLeft.setPosition(ARM_POSITION);
        armServoRight.setPosition(1 - ARM_POSITION);

        boxServo.setPosition(BOX_SERVO_POSITION);
    }
}
