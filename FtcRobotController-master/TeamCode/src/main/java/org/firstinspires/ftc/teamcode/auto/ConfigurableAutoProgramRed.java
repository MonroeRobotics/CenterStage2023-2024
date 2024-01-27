package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.util.ArmController;
import org.firstinspires.ftc.teamcode.util.AutoConfiguration;
import org.firstinspires.ftc.teamcode.util.HeadingHelper;
import org.firstinspires.ftc.vision.AprilTagHomer;
import org.firstinspires.ftc.vision.TeamPropDetection;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.Objects;

@Autonomous(name = "Configurable Red Board Auto", group = "Main")
@Config
public class ConfigurableAutoProgramRed extends LinearOpMode {

    //region Dashboard Variable Declarations

    //region Auto Timer

    public static double SPIKE_OUTTAKE_TIME = 1000; //Time Spike Pixel Outtakes In auto
    public static double BOARD_OUTTAKE_TIME = 500;//Time Board Pixel Outtakes in auto
    public static double PARK_TIME = 2000; //Time to go to park pos
    public static double APRIL_HOMER_LIMIT = 3000; //Failsafe for if apriltag homer has issues

    double waitTimer;


    //endregion

    public static double SPIKE_OUTTAKE_POWER = -0.3; //Stores the power of the reversed intake for spike pixel drop

    public static double CAMERA_EXPOSURE = 12;
    public static int CAMERA_GAIN = 255;
    //endregion

    SampleMecanumDrive drive;

    HeadingHelper headingHelper;

    //region Trajectory Declarations
    TrajectorySequence toSpikeMark;
    Trajectory toRedBoard;
    TrajectorySequence redBoardPark;
    Trajectory toPreTruss;
    Trajectory toPostTruss;
    //endregion
    ArmController armController;

    //region Intake Objects
    DcMotorEx intakeMotor;
    Servo intakeServo;
    //endregion

    //region Vision Objects
    TeamPropDetection propDetection;
    String screenSector;
    int targetTagId;
    AprilTagProcessor aprilTagDetector;
    VisionPortal visionPortal;
    AprilTagHomer aprilTagHomer;
    //endregion

    //region RR static coordinates

    //region red board spike locations
    Pose2d spikeLocation;

    Pose2d spikeLeft;
    Pose2d spikeCenter;
    Pose2d spikeRight;
    //endregion

    Pose2d beforeTrussCord = new Pose2d(-36, -12, Math.toRadians(180));
    Pose2d afterTrussCord = new Pose2d(30, -12, Math.toRadians(180));

    Pose2d startingDrivePose;

    Pose2d startingDrivePoseBoard = new Pose2d(10, -62, Math.toRadians(270));
    Pose2d startingDrivePoseAway = new Pose2d(-35, -62, Math.toRadians(270));


    //y was previously -35
    public static Pose2d centerRedBoardCord = new Pose2d(35, -36, Math.toRadians(180));
    public static Pose2d rightRedBoardCord = new Pose2d(35, -40, Math.toRadians(180));
    public static Pose2d leftRedBoardCord = new Pose2d(35, -32, Math.toRadians(180));
    public static Pose2d redBoardCord = new Pose2d(35, -38, Math.toRadians(180));
    public static Pose2d redParkCord;

    enum autoState {
        START,
        TO_SPIKE_MARK,
        VISION_SWITCH,
        PRE_TRUSS,
        POST_TRUSS,
        TO_BOARD,
        HOME_TAG,
        PLACE_BOARD,
        PARK,
        STOP
    }

    autoState queuedState = autoState.START;

    //region autoConfiguration variables
    Gamepad currentGamepad;
    Gamepad previousGamepad;
    AutoConfiguration autoConfiguration;
    //endregion

    @Override
    public void runOpMode() {

        autoConfiguration = new AutoConfiguration(telemetry, AutoConfiguration.AllianceColor.RED);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        drive = new SampleMecanumDrive(hardwareMap);

        headingHelper = new HeadingHelper(drive, hardwareMap, telemetry);

        armController = new ArmController(hardwareMap);

        armController.initArm();

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

        propDetection = new TeamPropDetection("red");

        visionPortal = VisionPortal.easyCreateWithDefaults(hardwareMap.get(WebcamName.class, "webcam"), aprilTagDetector, propDetection);
        visionPortal.setProcessorEnabled(aprilTagDetector, false);

        currentGamepad = new Gamepad();
        previousGamepad = new Gamepad();
        currentGamepad.copy(gamepad1);
        previousGamepad.copy(gamepad1);


        while(opModeInInit()){
            autoConfiguration.processInput(currentGamepad, previousGamepad);

            previousGamepad.copy(currentGamepad);
            currentGamepad.copy(gamepad1);
        }

        if(autoConfiguration.getStartPosition() == AutoConfiguration.StartPosition.BOARD){
            startingDrivePose = startingDrivePoseBoard;
        }
        else{
            startingDrivePose = startingDrivePoseAway;
        }

        drive.setPoseEstimate(startingDrivePose);


        while (opModeIsActive()) {
            telemetry.addData("Next auto State", queuedState);

            switch (queuedState) {
                case START:
                    if(autoConfiguration.getStartPosition() == AutoConfiguration.StartPosition.BOARD){
                        spikeLeft = new Pose2d(4,-40, Math.toRadians(315));
                        spikeCenter = new Pose2d(12,-34.5, Math.toRadians(270));
                        spikeRight = new Pose2d(19,-37, Math.toRadians(240));
                    }else{
                        spikeRight = new Pose2d(-33,-28, Math.toRadians(180));
                        spikeCenter = new Pose2d(-35,-33, Math.toRadians(270));
                        spikeLeft = new Pose2d(-42,-35, Math.toRadians(315));
                    }

                    screenSector = propDetection.getScreenSector();
                    if (screenSector != null) {
                        if (screenSector.equals("L")) {
                            spikeLocation = spikeLeft;
                            redBoardCord = leftRedBoardCord;
                            targetTagId = 4;
                        } else if (screenSector.equals("C")) {
                            spikeLocation = spikeCenter;
                            redBoardCord = centerRedBoardCord;
                            targetTagId = 5;
                        } else {
                            spikeLocation = spikeRight;
                            redBoardCord = rightRedBoardCord;
                            targetTagId = 6;
                        }

                        queuedState = autoState.TO_SPIKE_MARK;
                    }
                    waitTimer = System.currentTimeMillis() + autoConfiguration.getDelay();
                    break;
                case TO_SPIKE_MARK:
                    if (!drive.isBusy() && System.currentTimeMillis() > waitTimer) {
                        switch (autoConfiguration.getStartPosition()) { //Switch for Spike Mark Pathing Depending on Side of Board

                            //region Board Spike Mark Pathing
                            case BOARD:
                                if(!Objects.equals(screenSector, "L")) {
                                    toSpikeMark = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                                            .lineToLinearHeading(spikeLocation)
                                            .forward(12)
                                            .build();
                                    drive.followTrajectorySequenceAsync(toSpikeMark);
                                }

                                else{
                                    toSpikeMark = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                                            .back(12)
                                            .lineToLinearHeading(spikeLocation)
                                            .forward(12)
                                            .build();
                                    drive.followTrajectorySequenceAsync(toSpikeMark);

                                }
                                break;
                            //endregion

                            //region Away Spike Mark Pathing
                            case AWAY:
                                if(Objects.equals(screenSector, "R")) {
                                    toSpikeMark = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                                            .back(18)
                                            .lineToLinearHeading(spikeLocation)
                                            .forward(12)
                                            .build();
                                    drive.followTrajectorySequenceAsync(toSpikeMark);
                                }
                                if(Objects.equals(screenSector, "C")) {
                                    toSpikeMark = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                                            .lineToLinearHeading(spikeLocation)
                                            .forward(12)
                                            .strafeRight(16)
                                            .back(28)
                                            .build();
                                    drive.followTrajectorySequenceAsync(toSpikeMark);
                                }
                                else if (Objects.equals(screenSector, "L")) {
                                    final Pose2d[] tempCord = {drive.getPoseEstimate()};
                                    toSpikeMark = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                                            .back(12)
                                            .addDisplacementMarker(() ->{
                                                tempCord[0] = drive.getPoseEstimate();
                                            })
                                            .lineToLinearHeading(spikeLocation)
                                            .lineToLinearHeading(tempCord[0])
                                            .back(32)
                                            .build();
                                    drive.followTrajectorySequenceAsync(toSpikeMark);
                                }
                                break;
                            //endregion
                        }
                        if (autoConfiguration.isPurplePixelOnly()) {
                            queuedState = autoState.STOP;
                        } else {
                            queuedState = autoState.VISION_SWITCH;
                        }
                    }
                    break;
                case VISION_SWITCH:
                    if (!drive.isBusy()) {
                        visionPortal.setProcessorEnabled(propDetection, false);
                        visionPortal.setProcessorEnabled(aprilTagDetector, true);

                        if(autoConfiguration.getStartPosition() == AutoConfiguration.StartPosition.BOARD)
                        queuedState = autoState.TO_BOARD;

                        else queuedState = autoState.PRE_TRUSS;
                    }
                    break;
                case PRE_TRUSS:
                    if(!drive.isBusy()){
                        toPreTruss = drive.trajectoryBuilder(drive.getPoseEstimate())
                                .lineToLinearHeading(beforeTrussCord)
                                .build();
                        drive.followTrajectoryAsync(toPreTruss);
                        queuedState = autoState.POST_TRUSS;
                    }
                    break;
                case POST_TRUSS:
                    if(!drive.isBusy()){
                        toPostTruss = drive.trajectoryBuilder(drive.getPoseEstimate())
                                .lineToLinearHeading(afterTrussCord)
                                .build();
                        drive.followTrajectoryAsync(toPostTruss);
                        queuedState = autoState.TO_BOARD;
                    }
                    break;
                case TO_BOARD:
                    if (!drive.isBusy()) {
                        intakeMotor.setPower(0);
                        toRedBoard = drive.trajectoryBuilder(drive.getPoseEstimate())
                                .lineToLinearHeading(redBoardCord)
                                .build();
                        armController.switchArmState();
                        drive.followTrajectoryAsync(toRedBoard);
                        queuedState = autoState.HOME_TAG;
                    }
                    break;
                case HOME_TAG:
                    if (!drive.isBusy()) {
                        aprilTagHomer.changeTarget(targetTagId);
                        aprilTagHomer.updateDrive();
                        waitTimer = System.currentTimeMillis() + APRIL_HOMER_LIMIT;
                        queuedState = autoState.PLACE_BOARD;
                    }
                    break;
                case PLACE_BOARD:
                    if (aprilTagHomer.inRange() || System.currentTimeMillis() > waitTimer) {
                        armController.startOuttake();
                        armController.startOuttake();
                        waitTimer = System.currentTimeMillis() + BOARD_OUTTAKE_TIME;
                        queuedState = autoState.PARK;
                        break;
                    }
                    if (aprilTagHomer.getCurrentTagPose() != null) {
                        telemetry.addData("Tag X:", aprilTagHomer.getCurrentTagPose().x);
                        telemetry.addData("Tag Y:", aprilTagHomer.getCurrentTagPose().y);
                        telemetry.addData("Tag Yaw:", aprilTagHomer.getCurrentTagPose().yaw);

                    } else {
                        telemetry.addLine("No Tag Detected");
                    }

                    aprilTagHomer.updateDrive();
                    break;
                case PARK:
                    if (!drive.isBusy() && System.currentTimeMillis() > waitTimer) {
                        //Trajectory to Park Pos

                        if(autoConfiguration.getParkSide() == AutoConfiguration.ParkSide.SIDE){
                            redParkCord = new Pose2d(48, -64, Math.toRadians(180));
                        }else{
                            redParkCord = new Pose2d(40, -20, Math.toRadians(180));
                        }

                        redBoardPark = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                                .forward(5)
                                .addDisplacementMarker(() -> {
                                    armController.switchArmState();
                                    armController.setSlideHeight(-10);
                                })
                                .lineToLinearHeading(redParkCord)
                                .build();
                        //Start Following Trajectory
                        drive.followTrajectorySequenceAsync(redBoardPark);




                        waitTimer = System.currentTimeMillis() + PARK_TIME;
                        queuedState = autoState.STOP;
                    }
                    break;
                case STOP:
                    if (!drive.isBusy()) {
                    }
            }

            headingHelper.loopMethod();

            telemetry.update();

            drive.update();

            armController.updateArm();
        }
    }
}