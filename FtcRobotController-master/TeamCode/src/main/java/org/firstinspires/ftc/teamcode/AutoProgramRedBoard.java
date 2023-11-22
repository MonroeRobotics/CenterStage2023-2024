package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
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

    //region Dashboard Static Variable Declarations

    //region Slide Variables
    public static int INIT_SLIDE_HEIGHT = 20;
    public static int PLACEMENT_SLIDE_HEIGHT = 540;
    public static double SLIDE_POWER = .5;
    public static int SLIDE_MAX_VELO = 2000;
    //endregion

    //region Arm Variables
    public static double ARM_POSITION = .05;
    //endregion

    //endregion

    SampleMecanumDrive drive;

    //region Trajectory Declarations
    Trajectory toSpikeMark;
    Trajectory toRedBoard;
    Trajectory redBoardPark;
    //endregion




    //region Arm Objects
    DcMotorEx intakeMotor;
    Servo intakeServo;
    CRServo outtakeServo;
    Servo boxServo;
    DcMotorEx leftLinear;
    DcMotorEx rightLinear;
    Servo armServoLeft;
    Servo armServoRight;
    //endregion

    double waitTimer;



    //region Vision Objects
    TeamPropDetection propDetection = new TeamPropDetection(telemetry);
    String screenSector;
    AprilTagProcessor aprilTagDetector;
    VisionPortal visionPortal;
    AprilTagHomer aprilTagHomer;
    //endregion

    //region static coordinates

    //region red board spike locations
    Pose2d spikeLocation;
    Pose2d spikeOne = new Pose2d(10,-30, Math.toRadians(180));
    Pose2d spikeTwo = new Pose2d(20,-24, Math.toRadians(180));
    Pose2d spikeThree = new Pose2d(13,-30, Math.toRadians(0));
    //endregion

    Pose2d redBoardCord = new Pose2d(48, -35, Math.toRadians(180));
    Pose2d redParkCord = new Pose2d(48, -60, Math.toRadians(180));
    Pose2d blueBoardCord = new Pose2d(48, 35, Math.toRadians(180));
    Pose2d blueParkCord = new Pose2d(48, 60, Math.toRadians(180));

    Pose2d redBoardCenterSpikeMark = new Pose2d(13, -30, Math.toRadians(0));
    Pose2d redBoardCenterSpikeMarkStart = new Pose2d(10,62, Math.toRadians(90));
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

    autoState currentState = autoState.START;



    @Override
    public void init() {
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
        armServoLeft.setPosition(ARM_POSITION);
        armServoRight.setPosition(1 - ARM_POSITION);
        boxServo.setPosition(BOX_SERVO_POSITION);
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


        //get webcam stuff here

        drive = new SampleMecanumDrive(hardwareMap);

        drive.setPoseEstimate(new Pose2d(10, 62, Math.toRadians(90)));



        visionPortal = VisionPortal.easyCreateWithDefaults(hardwareMap.get(WebcamName.class, "webcam"), aprilTagDetector, propDetection);
        visionPortal.setProcessorEnabled(aprilTagDetector, false);

        toSpikeMark = drive.trajectoryBuilder(drive.getPoseEstimate())
            .lineToLinearHeading(spikeLocation)
                //place purple pixel on spike line
                .addDisplacementMarker(() -> {
                    intake.setPower(-1);
                    outakeServo.setPosition(-.5);
                })
                .lineToLinearHeading(redBoardCord)
                //go to april tag indicated by spike marker location
                .addDisplacementMarker(() -> {
                    aprilTagDetector.getDetections();
                    aprilTagHomer.getCurrentTagPose();
                    aprilTagHomer.updateDrive();
                })
                .lineToLinearHeading(redParkCord)
                .build();

        drive.followTrajectoryAsync(toSpikeMark);

    }

    @Override
    public void loop() {
        switch ()




        drive.update();
    }
}
