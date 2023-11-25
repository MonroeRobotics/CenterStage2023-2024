package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.vision.AprilTagHomer;
import org.firstinspires.ftc.vision.TeamPropDetection;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.Objects;

@Autonomous(name = "Blue Board Auto", group = "Main")
@Config
public class AutoProgramBlueBoard extends OpMode {

    //region Dashboard Variable Declarations

    //region Auto Timer

    public static double SPIKE_OUTTAKE_TIME = 1000; //Time Spike Pixel Outtakes In auto
    public static double BOARD_OUTTAKE_TIME = 1000;//Time Board Pixel Outtakes in auto
    public static double PARK_TIME = 2000; //Time to go to park pos
    public static double APRIL_HOMER_LIMIT = 3000; //Failsafe for if apriltag homer has issues

    double waitTimer;


    //endregion

    //region Slide Variables
    public static int INIT_SLIDE_HEIGHT = 20;
    public static int PLACEMENT_SLIDE_HEIGHT = 450;//Slide height for placing pixels on board
    public static double SLIDE_POWER = .5; //Max Slide Power
    public static int SLIDE_MAX_VELO = 2000;
    //endregion

    //region Arm Variables
    public static double ARM_SERVO_FORWARD = 0.04;//Stores Value of Arm intake Position
    public static double ARM_SERVO_BACKWARD = 0.7;//Stores Value of Arm outtake Position

    public static double BOX_SERVO_FORWARD = 1; //Stores Value of Box intake Position
    public static double BOX_SERVO_BACKWARD = 0.3;//Stores Value of Box outtake Position
    //endregion

    public static double SPIKE_OUTTAKE_POWER = -0.3; //Stores the power of the reversed intake for spike pixel drop

    //endregion

    SampleMecanumDrive drive;

    //region Trajectory Declarations
    Trajectory toSpikeMark;
    Trajectory toBlueBoard;
    Trajectory blueBoardPark;
    //endregion


    //region Intake Objects
    DcMotorEx intakeMotor;
    Servo intakeServo;
    //endregion

    //region Arm Objects
    CRServo outtakeServo;
    Servo boxServo;
    DcMotorEx leftLinear;
    DcMotorEx rightLinear;
    Servo armServoLeft;
    Servo armServoRight;
    //endregion

    //region Vision Objects
    TeamPropDetection propDetection = new TeamPropDetection("blue");
    String screenSector;
    int targetTagId;
    AprilTagProcessor aprilTagDetector;
    VisionPortal visionPortal;
    AprilTagHomer aprilTagHomer;
    //endregion

    //region RR static coordinates

    //region blue board spike locations
    Pose2d spikeLocation;

    public static Pose2d spikeLeft = new Pose2d(10,30, Math.toRadians(180));
    public static Vector2d spikeLeftSpline = new Vector2d(11,32);

    static Pose2d spikeCenter = new Pose2d(20,25.5, Math.toRadians(180));
    public static Pose2d spikeRight = new Pose2d(32.5,30, Math.toRadians(180));
    //endregion

    public static Pose2d STARTING_DRIVE_POS = new Pose2d(10, 62, Math.toRadians(90));

    //y was previously -35
    public static Pose2d centerBlueBoardCord = new Pose2d(35, 36, Math.toRadians(180));
    public static Pose2d rightBlueBoardCord = new Pose2d(35, 40, Math.toRadians(180));
    public static Pose2d leftBlueBoardCord = new Pose2d(35, 32, Math.toRadians(180));
    public static Pose2d blueBoardCord = new Pose2d(35, 38, Math.toRadians(180));
    public static  Pose2d blueParkCord = new Pose2d(48, 60, Math.toRadians(180));

    /*
    Pose2d blueBoardCord = new Pose2d(48, 35, Math.toRadians(180));
    Pose2d blueParkCord = new Pose2d(48, 60, Math.toRadians(180));
    */
    //endregion


    enum autoState {
        START,
        TO_SPIKE_MARK,
        OUTTAKE_SPIKE,
        TO_BOARD,
        HOME_TAG,
        PLACE_BOARD,
        PARK,
        STOP
    }

    autoState queuedState = autoState.START;



    @Override
    public void init() {

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        drive = new SampleMecanumDrive(hardwareMap);

        drive.setPoseEstimate(STARTING_DRIVE_POS);

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

        leftLinear.setTargetPosition(INIT_SLIDE_HEIGHT);
        rightLinear.setTargetPosition(INIT_SLIDE_HEIGHT);

        leftLinear.setPower(SLIDE_POWER);
        rightLinear.setPower(SLIDE_POWER);

        leftLinear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightLinear.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftLinear.setVelocity(SLIDE_MAX_VELO);
        rightLinear.setVelocity(SLIDE_MAX_VELO);
        //endregion

        //region Initial Servo Pos
        armServoLeft.setPosition(ARM_SERVO_FORWARD);
        armServoRight.setPosition(1 - ARM_SERVO_FORWARD);
        boxServo.setPosition(BOX_SERVO_FORWARD);
        //endregion

        //endregion

        //region Intake Init
        //region Intake Hardware Map
        intakeMotor = hardwareMap.get(DcMotorEx.class, "intakeMotor");
        intakeServo = hardwareMap.get(Servo.class, "intakeServo");
        //endregion

        //region Intake Settings
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeServo.setPosition(1);
        //endregion
        //endregion

        aprilTagDetector = AprilTagProcessor.easyCreateWithDefaults();

        aprilTagHomer = new AprilTagHomer(aprilTagDetector, drive);

        propDetection = new TeamPropDetection("blue");

        visionPortal = VisionPortal.easyCreateWithDefaults(hardwareMap.get(WebcamName.class, "webcam"), aprilTagDetector, propDetection);
        visionPortal.setProcessorEnabled(aprilTagDetector, false);

    }

    @Override
    public void loop() {
        telemetry.addData("Next auto State", queuedState);

        switch (queuedState){
            case START:
                screenSector = propDetection.getScreenSector();
                if(screenSector != null) {
                    if (screenSector.equals("L")) {
                        spikeLocation = spikeLeft;
                        blueBoardCord = leftBlueBoardCord;
                        targetTagId = 1;
                    } else if (screenSector.equals("C")) {
                        spikeLocation = spikeCenter;
                        blueBoardCord = centerBlueBoardCord;
                        targetTagId = 2;
                    } else {
                        spikeLocation = spikeRight;
                        blueBoardCord = rightBlueBoardCord;
                        targetTagId = 3;
                    }

                    queuedState = autoState.TO_SPIKE_MARK;
                }
                break;
            case TO_SPIKE_MARK:
                if(!drive.isBusy() & !Objects.equals(screenSector, "L")) {
                    toSpikeMark = drive.trajectoryBuilder(drive.getPoseEstimate())
                            .lineToLinearHeading(spikeLocation)
                            .build();
                    drive.followTrajectoryAsync(toSpikeMark);
                    queuedState = autoState.OUTTAKE_SPIKE;
                }
                else if (!drive.isBusy()) {
                    toSpikeMark = drive.trajectoryBuilder(drive.getPoseEstimate())
                            .back(24)
                            .splineTo(spikeLeftSpline, Math.toRadians(0))
                            .build();
                    drive.followTrajectoryAsync(toSpikeMark);
                    queuedState = autoState.OUTTAKE_SPIKE;
                }
                break;
            case OUTTAKE_SPIKE:
                if(!drive.isBusy()){
                    visionPortal.setProcessorEnabled(propDetection, false);
                    visionPortal.setProcessorEnabled(aprilTagDetector, true);
                    waitTimer = System.currentTimeMillis() + SPIKE_OUTTAKE_TIME;
                    intakeMotor.setPower(-SPIKE_OUTTAKE_POWER);
                    queuedState = autoState.TO_BOARD;
                }
                break;
            case TO_BOARD:
                if(!drive.isBusy() && System.currentTimeMillis() >= waitTimer){
                    intakeMotor.setPower(0);
                    toBlueBoard = drive.trajectoryBuilder(toSpikeMark.end())
                            .lineToLinearHeading(blueBoardCord)
                            .build();
                    leftLinear.setTargetPosition(PLACEMENT_SLIDE_HEIGHT);
                    rightLinear.setTargetPosition(PLACEMENT_SLIDE_HEIGHT);
                    armServoLeft.setPosition(ARM_SERVO_BACKWARD);
                    armServoRight.setPosition(1 - ARM_SERVO_BACKWARD);
                    boxServo.setPosition(BOX_SERVO_BACKWARD);
                    drive.followTrajectoryAsync(toBlueBoard);
                    queuedState = autoState.HOME_TAG;
                }
                break;
            case HOME_TAG:
                if(!drive.isBusy()){
                    aprilTagHomer.changeTarget(targetTagId);
                    aprilTagHomer.updateDrive();
                    waitTimer = System.currentTimeMillis() + APRIL_HOMER_LIMIT;
                    queuedState = autoState.PLACE_BOARD;
                }
                break;
            case PLACE_BOARD:
                if(aprilTagHomer.inRange() || System.currentTimeMillis() > waitTimer){
                    outtakeServo.setPower(1);
                    waitTimer = System.currentTimeMillis() + BOARD_OUTTAKE_TIME;
                    queuedState = autoState.PARK;
                    break;
                }
                if(aprilTagHomer.getCurrentTagPose() != null) {
                    telemetry.addData("Tag X:", aprilTagHomer.getCurrentTagPose().x);
                    telemetry.addData("Tag Y:", aprilTagHomer.getCurrentTagPose().y);
                    telemetry.addData("Tag Yaw:", aprilTagHomer.getCurrentTagPose().yaw);

                }
                else{telemetry.addLine("No Tag Detected");
                }

                aprilTagHomer.updateDrive();
                break;
            case PARK:
                if(!drive.isBusy() && System.currentTimeMillis() > waitTimer){
                    outtakeServo.setPower(0);
                    //Trajectory to Park Pos
                    blueBoardPark = drive.trajectoryBuilder(drive.getPoseEstimate())
                            .lineToLinearHeading(blueParkCord)
                            .build();
                    //Start Following Trajectory
                    drive.followTrajectoryAsync(blueBoardPark);
                    //Put slide and arm back to intake position
                    armServoLeft.setPosition(ARM_SERVO_FORWARD);
                    armServoRight.setPosition(1 - ARM_SERVO_FORWARD);
                    boxServo.setPosition(BOX_SERVO_FORWARD);
                    waitTimer = System.currentTimeMillis() + PARK_TIME;
                    queuedState = autoState.STOP;
                }
                break;
            case STOP:
                if(!drive.isBusy()){
                    leftLinear.setTargetPosition(5);
                    rightLinear.setTargetPosition(5);

                    if(System.currentTimeMillis() > waitTimer){
                        requestOpModeStop();
                    }
                }

        }




        telemetry.update();

        drive.update();
    }
}
