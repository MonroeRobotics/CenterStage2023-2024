package org.firstinspires.ftc.teamcode.testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.util.ArmController;
import org.firstinspires.ftc.teamcode.util.PixelGamepadDetector;

@TeleOp(name="Arm Tuner", group = "Main")
@Config
public class ArmTuner extends OpMode {

    //region variable declarations


    //region FTC Dashboard config variables
    public static double ARM_POSITION = 0;
    public static double OUTTAKE_POWER = 0;
    public static int SLIDE_HEIGHT = 30;

    public static double BOX_POS = 0.25;

    public static int RIGGING_POSITION = 20;

    //endregion

    //region Intake Variables
    public static double INTAKE_POWER = .8; //Power of Intake Motor
    public static double INTAKE_POSITION = .3; //Position of Intake Servo
    boolean intakeActive = false;
    boolean reverseIntake = false;

    public static boolean controllerMode = true;

    public static double PIXEL_DETECTION_DISTANCE = 1; //Distance from color sensor to pixel for detection (CM)
    public double reverseTimer = 0; //timer for reversing intake
    public static double REVERSE_TIME = 1000; //How long to Reverse intake


    //endregion


    public static int RIGGING_EXTENDED_POS = 14500;

    //endregion

    //region Declare Objects

    //region EndGame Objects
    DcMotorEx hangMotor;

    Servo droneServo;

    //endregion

    ArmController armController;

    //region Intake Objects
    DcMotor intakeMotor;
    Servo intakeServo;

    PixelGamepadDetector pixelGamepadDetector;

    RevColorSensorV3 colorSensor1;
    RevColorSensorV3 colorSensor2;

    //endregion

    //Creates Gamepad objects to store current and previous Gamepad states
    Gamepad currentGamepad1;
    Gamepad currentGamepad2;

    Gamepad previousGamepad1;
    Gamepad previousGamepad2;
    //endregion

    double droneTimer = 0;


    @Override
    public void init() {
        //Sets up telemetry to work with FTC Dashboard
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        //region Intake Init
        //region Intake Hardware Map
        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
        intakeServo = hardwareMap.get(Servo.class, "intakeServo");
        //endregion

        armController = new ArmController(hardwareMap);
        armController.initArm();

       /* //region Intake Settings
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeServo.setPosition(1);
        //endregion*/
        //endregion

       /* //region Rigging Init
        hangMotor = hardwareMap.get(DcMotorEx.class,"hangMotor");
        hangMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        hangMotor.setPower(1);
        hangMotor.setTargetPosition(0);
        hangMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        hangMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        hangMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        droneServo = hardwareMap.get(Servo.class, "droneServo");
        droneServo.setPosition(0);
        //endregion*/

      /*  colorSensor1 = hardwareMap.get(RevColorSensorV3.class,"colorSensor1");
        colorSensor2 = hardwareMap.get(RevColorSensorV3.class, "colorSensor2");

        currentGamepad1 = new Gamepad();
        currentGamepad2 = new Gamepad();

        previousGamepad1 = new Gamepad();
        previousGamepad2 = new Gamepad();*/

        //See util.PixelGamepadDetector
//        pixelGamepadDetector = new PixelGamepadDetector(this.gamepad1, this.gamepad2, colorSensor1, colorSensor2);
    }
    @Override
    public void loop() {

       /* if (controllerMode) {
            if (droneTimer == 0) {
                droneTimer = System.currentTimeMillis() + 100000;
            }

            if (currentGamepad2.ps && !previousGamepad2.ps) {
                droneTimer = 1;
            }

            if (currentGamepad2.options && System.currentTimeMillis() >= droneTimer) {
                hangMotor.setPower(1);

                hangMotor.setTargetPosition(5020);
            }
            if (currentGamepad2.touchpad && System.currentTimeMillis() >= droneTimer) {
                droneServo.setPosition(1);
            }

            //region Arm Logic

            //Increases or Decreases Slide Stage on Dpad up or down within [0,7]
            if (currentGamepad2.dpad_up && !previousGamepad2.dpad_up) {
                armController.changeStage(1);
            } else if (currentGamepad2.dpad_down && !previousGamepad2.dpad_down) {
                armController.changeStage(-1);
            }

            //Changes Arm State on Triangle Press
            if (currentGamepad2.triangle && !previousGamepad2.triangle) {
                armController.switchArmState();
            }
            //Resets Arm to Intake Position on square press
            else if (currentGamepad2.square && !previousGamepad2.square) {
                armController.startOuttake();
            }


            //Manual Jog For Slides (In Case of emergency)
            if (currentGamepad2.right_bumper && currentGamepad2.left_bumper) {
                if (currentGamepad2.left_trigger >= 0.1) {
                    armController.setSlideHeight(armController.getSlideHeight() - 10);
                } else if (currentGamepad2.right_trigger >= 0.1) {
                    armController.setSlideHeight(armController.getSlideHeight() + 10);
                }
            }
            //endregion

            //region Intake Logic

            //On cross start intake (Only if arm is in intake position)
            if (armController.getCurrentArmState() == ArmController.ArmState.INTAKE) {
                if (currentGamepad2.cross && !previousGamepad2.cross) {
                    intakeActive = true;
                } else if (currentGamepad2.circle && !previousGamepad2.circle) {
                    intakeActive = false;
                    reverseIntake = false;
                    armController.setOuttakePower(0);
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
                armController.setOuttakePower(0);
            }

            if (intakeActive) {

                intakeMotor.setPower(INTAKE_POWER);
                intakeServo.setPosition(INTAKE_POSITION);

                armController.setOuttakePower(-1);

                //If two pixels are detected within specified distance, start reversal of intake
                if (colorSensor1.getDistance(DistanceUnit.CM) <= PIXEL_DETECTION_DISTANCE && colorSensor2.getDistance(DistanceUnit.CM) <= PIXEL_DETECTION_DISTANCE) {
                    intakeActive = false;
                    reverseIntake = true;
                    reverseTimer = System.currentTimeMillis() + REVERSE_TIME;
                }
            }
            //Checks if reverse is on and timer is still on
            else if (reverseIntake && reverseTimer > System.currentTimeMillis()) {
                intakeServo.setPosition(INTAKE_POSITION);
                intakeMotor.setPower(-INTAKE_POWER);
                armController.setOuttakePower(0);
            } else {
                intakeMotor.setPower(0);
                intakeServo.setPosition(1);
            }
            //endregion

            //region Rigging Logic
            if (currentGamepad2.left_stick_button && !previousGamepad2.left_stick_button) {
                hangMotor.setPower(1);
                hangMotor.setTargetPosition(RIGGING_EXTENDED_POS);
            }
            if (currentGamepad2.right_stick_button) {
                hangMotor.setTargetPosition(hangMotor.getTargetPosition() - 80);

            }
            //endregion

            armController.updateArm();

            pixelGamepadDetector.updateControllerColors();

            previousGamepad1.copy(currentGamepad1);
            previousGamepad2.copy(currentGamepad2);

            currentGamepad1.copy(gamepad1);
            currentGamepad2.copy(gamepad2);
        }*/
            armController.setArmPos(ARM_POSITION);
            armController.setSlideHeight(SLIDE_HEIGHT);
            armController.setOuttakePower(OUTTAKE_POWER);
            armController.setBoxPos(BOX_POS);

            armController.updateArmABS();

//            hangMotor.setPower(1);
//            hangMotor.setTargetPosition(RIGGING_POSITION);
    }


}
