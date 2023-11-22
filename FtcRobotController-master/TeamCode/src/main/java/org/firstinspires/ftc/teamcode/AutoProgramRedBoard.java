package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
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

@Autonomous(name = "Auto Program Main", group = "Main")
@Config
public class AutoProgramRedBoard extends OpMode {

    //region Dashboard Variable Declarations

    //region Auto Timer

    public static double SPIKE_OUTTAKE_TIME = 1000; //Time Spike Pixel Outtakes In auto
    public static double BOARD_OUTTAKE_TIME = 1000;//Time Board Pixel Outtakes in auto
    double waitTimer;

    //endregion

    //region Slide Variables
    public static int INIT_SLIDE_HEIGHT = 20;
    public static int PLACEMENT_SLIDE_HEIGHT = 540;//Slide height for placing pixels on board
    public static double SLIDE_POWER = .5;//Max Slide Power
    public static int SLIDE_MAX_VELO = 2000;
    //endregion

    //region Arm Variables
    public static double ARM_SERVO_FORWARD = 0.04;//Stores Value of Arm intake Position
    public static double ARM_SERVO_BACKWARD = 0.7;//Stores Value of Arm outtake Position

    public static double BOX_SERVO_FORWARD = 1; //Stores Value of Box intake Position
    public static double BOX_SERVO_BACKWARD = 0.3;//Stores Value of Box outtake Position
    //endregion

    public static double SPIKE_OUTTAKE_POWER = 0.3; //Stores the power of the reversed intake for spike pixel drop

    //endregion

    SampleMecanumDrive drive;

    //region Trajectory Declarations
    Trajectory toSpikeMark;
    Trajectory toRedBoard;
    Trajectory redBoardPark;
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
    TeamPropDetection propDetection = new TeamPropDetection(telemetry);
    String screenSector;
    int targetTagId;
    AprilTagProcessor aprilTagDetector;
    VisionPortal visionPortal;
    AprilTagHomer aprilTagHomer;
    //endregion

    //region RR static coordinates

    //region red board spike locations
    Pose2d spikeLocation;
    public static Pose2d spikeLeft = new Pose2d(10,-30, Math.toRadians(180));
    public static Pose2d spikeCenter = new Pose2d(20,-24, Math.toRadians(180));
    public static Pose2d spikeRight = new Pose2d(32,-30, Math.toRadians(180));
    //endregion

    public static Pose2d STARTING_DRIVE_POS = new Pose2d(10, -62, Math.toRadians(270));

    public static Pose2d redBoardCord = new Pose2d(43, -35, Math.toRadians(180));
    public static  Pose2d redParkCord = new Pose2d(48, -60, Math.toRadians(180));

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
        PARK
    }

    autoState queuedState = autoState.START;



    @Override
    public void init() {
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

        propDetection = new TeamPropDetection(telemetry);

        visionPortal = VisionPortal.easyCreateWithDefaults(hardwareMap.get(WebcamName.class, "webcam"), aprilTagDetector, propDetection);
        visionPortal.setProcessorEnabled(aprilTagDetector, false);

    }

    @Override
    public void loop() {
        switch (queuedState){
            case START:
                if(screenSector != null) {
                    screenSector = propDetection.getScreenSector();

                    if (screenSector == "L") {
                        spikeLocation = spikeLeft;
                        targetTagId = 4;
                    } else if (screenSector == "C") {
                        spikeLocation = spikeCenter;
                        targetTagId = 5;
                    } else {
                        spikeLocation = spikeRight;
                        targetTagId = 6;
                    }

                    queuedState = autoState.TO_SPIKE_MARK;
                }
                break;
            case TO_SPIKE_MARK:
                if(!drive.isBusy()) {
                    toSpikeMark = drive.trajectoryBuilder(drive.getPoseEstimate())
                            .lineToLinearHeading(spikeLocation)
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
                    toRedBoard = drive.trajectoryBuilder(toSpikeMark.end())
                            .lineToLinearHeading(redBoardCord)
                            .build();
                    leftLinear.setTargetPosition(PLACEMENT_SLIDE_HEIGHT);
                    rightLinear.setTargetPosition(PLACEMENT_SLIDE_HEIGHT);
                    armServoLeft.setPosition(ARM_SERVO_BACKWARD);
                    armServoRight.setPosition(1 - ARM_SERVO_BACKWARD);
                    boxServo.setPosition(BOX_SERVO_BACKWARD);
                    drive.followTrajectoryAsync(toRedBoard);
                    queuedState = autoState.HOME_TAG;
                }
                break;
            case HOME_TAG:
                if(!drive.isBusy()){
                    aprilTagHomer.changeTarget(targetTagId);
                    aprilTagHomer.updateDrive();
                    queuedState = autoState.PLACE_BOARD;
                }
                break;
            case PLACE_BOARD:
                if(aprilTagHomer.inRange()){
                    outtakeServo.setPower(1);
                    waitTimer = System.currentTimeMillis() + BOARD_OUTTAKE_TIME;
                    queuedState = autoState.PLACE_BOARD;
                    break;
                }
                aprilTagHomer.updateDrive();
                break;
            case PARK:
                if(!drive.isBusy() && System.currentTimeMillis() > waitTimer){
                    outtakeServo.setPower(0);
                    //Trajectory to Park Pos
                    toRedBoard = drive.trajectoryBuilder(toSpikeMark.end())
                            .lineToLinearHeading(redBoardCord)
                            .build();
                    //Start Following Trajectory
                    drive.followTrajectoryAsync(toRedBoard);
                    //Put slide and arm back to intake position
                    leftLinear.setTargetPosition(5);
                    rightLinear.setTargetPosition(5);
                    armServoLeft.setPosition(ARM_SERVO_FORWARD);
                    armServoRight.setPosition(1 - ARM_SERVO_FORWARD);
                    boxServo.setPosition(BOX_SERVO_FORWARD);
                }
                break;

        }

        drive.update();
    }
}
