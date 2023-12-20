package org.firstinspires.ftc.teamcode.teleop;

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
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.util.PixelGamepadDetector;

@TeleOp(name="Main Drive", group = "Main")
@Config
public class main extends OpMode {

    //region variable declarations


    //region Intake Variables
    public static double INTAKE_POWER = .8; //Power of Intake Motor
    public static double INTAKE_POSITION = .3; //Position of Intake Servo
    boolean intakeActive = false;
    boolean reverseIntake = false;

    public static double PIXEL_DETECTION_DISTANCE = 1; //Distance from color sensor to pixel for detection (CM)
    public double reverseTimer = 0; //timer for reversing intake
    public static double REVERSE_TIME = 1000; //How long to Reverse intake


    //endregion

    //region Drive Variables

    public static double drivePower = 0.8;
    double xPower;
    double yPower;
    double headingPower;
    //endregion

    public static int RIGGING_EXTENDED_POS = 14500;

    //endregion

    //region Declare Objects

    SampleMecanumDrive drive;


    //region EndGame Objects
    DcMotorEx hangMotor;

    Servo droneServo;

    //endRegion

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

    double droneTimer = 0;


    @Override
    public void init() {
        //Sets up telemetry to work with FTC Dashboard
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        //creates and sets up RR Mecanum Drive
        drive = new SampleMecanumDrive(hardwareMap);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

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

        //region Rigging Init
        hangMotor = hardwareMap.get(DcMotorEx.class,"hangMotor");
        hangMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        hangMotor.setPower(1);
        hangMotor.setTargetPosition(0);
        hangMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        hangMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        hangMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        droneServo = hardwareMap.get(Servo.class, "droneServo");
        droneServo.setPosition(0);



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
        if (droneTimer==0){
            droneTimer = System.currentTimeMillis() + 100000;
        }

        if(currentGamepad2.ps && !previousGamepad2.ps){
            droneTimer = 1;
        }

        if (currentGamepad2.options && System.currentTimeMillis() >= droneTimer){
            hangMotor.setPower(1);

            hangMotor.setTargetPosition(5020);
        }
        if (currentGamepad2.touchpad && System.currentTimeMillis() >= droneTimer){
            droneServo.setPosition(1);
        }

        //region Drive Logic
        xPower = -gamepad1.left_stick_y;
        yPower = -gamepad1.left_stick_x;
        headingPower = -gamepad1.right_stick_x;


        double scaledPower = 1 - currentGamepad1.right_trigger;

        if (currentGamepad1.right_trigger >= 0.1 && !(previousGamepad1.right_trigger >= 0.1) && drivePower <= 0.8) {
            drivePower += 0.2;
        } else if (currentGamepad1.left_trigger >= 0.1 && !(previousGamepad1.left_trigger >= 0.1) && drivePower >= 0.8) {
            drivePower -= 0.2;
        }

        xPower *= drivePower;
        yPower *= drivePower;
        headingPower *= drivePower;


        if (currentGamepad1.dpad_up) {
            xPower = drivePower;
            yPower = 0;
            headingPower = 0;
        } else if (currentGamepad1.dpad_down) {
            xPower = -drivePower;
            yPower = 0;
            headingPower = 0;
        } else if (currentGamepad1.dpad_left) {
            xPower = 0;
            yPower = -drivePower;
            headingPower = 0;
        } else if (currentGamepad1.dpad_right) {
            xPower = 0;
            yPower = drivePower;
            headingPower = 0;

        }

        if (currentGamepad1.right_bumper) {
            xPower *= scaledPower;
            yPower *= scaledPower;
            headingPower *= scaledPower;
        }

        drive.setWeightedDrivePower(new Pose2d(
                xPower,
                yPower,
                headingPower
        ));

        //endregion

        //region Arm Logic

        //Increases or Decreases Slide Stage on Dpad up or down within [0,7]
        if (currentGamepad2.dpad_up && !previousGamepad2.dpad_up) {
            armController.changeStage(1);
        } else if (currentGamepad2.dpad_down && !previousGamepad2.dpad_down) {
            armController.changeStage(-1);
        }

        //Changes Arm State on Triangle Press
        if (currentGamepad2.triangle && !previousGamepad2.triangle) {
            armController.changeArmState();
        }
        //Resets Arm to Intake Position on square press
        else if (currentGamepad2.square && !previousGamepad2.square && outtakeTimer <= System.currentTimeMillis()) {
            outtakeTimer = System.currentTimeMillis() + OUTTAKE_TIME;
            outtakeServo.setPower(1);
        }
        else if (currentGamepad2.square && !previousGamepad2.square && outtakeTimer >= System.currentTimeMillis() && outtakeTimer <= (System.currentTimeMillis() + (OUTTAKE_TIME * 2))){
            outtakeTimer += OUTTAKE_TIME;
            outtakeServo.setPower(1);
        }

        if(outtakeTimer <= System.currentTimeMillis() && currentArmState == ArmState.OUTTAKE_READY){
            outtakeServo.setPower(0);
        }

        //Manual Jog For Slides (In Case of emergency)
        if(currentGamepad2.right_bumper && currentGamepad2.left_bumper){
            if(currentGamepad2.left_trigger >= 0.1){
                armController.setSlideHeight(armController.getSlideHeight() -= 10);
            }
            else if(currentGamepad2.right_trigger >= 0.1){
                armController.setSlideHeight(armController.getSlideHeight() += 10);
            }
        }


        /*
        //Changes Arm State back to intake once outtake timer runs out
        if (currentArmState == ArmState.OUTTAKE_ACTIVE && outtakeTimer < System.currentTimeMillis()) {
            currentArmState = ArmState.INTAKE;
            changeArmState();
        }*/




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

        //region Rigging Logic
        if(currentGamepad2.left_stick_button && !previousGamepad2.left_stick_button){
            hangMotor.setPower(1);

            hangMotor.setTargetPosition(RIGGING_EXTENDED_POS);
        }
        if(currentGamepad2.right_stick_button){
            leftLinear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightLinear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            hangMotor.setTargetPosition(hangMotor.getTargetPosition() - 80);
        }
        //endregion

        drive.update();

        pixelGamepadDetector.updateControllerColors();

        previousGamepad1.copy(currentGamepad1);
        previousGamepad2.copy(currentGamepad2);

        currentGamepad1.copy(gamepad1);
        currentGamepad2.copy(gamepad2);

    }


}
