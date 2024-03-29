package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
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

@Autonomous(name = "Configurable Blue Auto", group = "Main")
@Config
public class ConfigurableAutoProgramBlue extends LinearOpMode {

    //region Dashboard Variable Declarations

    //region Auto Timer
    public static double SPIKE_OUTTAKE_TIME = 1000; //Time Spike Pixel Outtakes In auto
    public static double BOARD_OUTTAKE_TIME = 800;//Time Board Pixel Outtakes in auto
    public static int WHITE_INTAKE_TIME = 3000;
    public static double PARK_TIME = 2000; //Time to go to park pos
    public static double APRIL_HOMER_LIMIT = 1500; //Failsafe for if apriltag homer has issues

    public static double ERROR_THRESH = 10;
    double waitTimer;
    //endregion

    public static double SPIKE_OUTTAKE_POWER = -0.3; //Stores the power of the reversed intake for spike pixel drop

    //endregion

    int autoCycleCount = 0;



    SampleMecanumDrive drive;

    HeadingHelper headingHelper;

    //region Trajectory Declarations
    TrajectorySequence toSpikeMark;
    Trajectory toRedBoard;
    TrajectorySequence blueBoardPark;
    Trajectory toPreTruss;
    Trajectory toPostTruss;

    TrajectorySequence toWhiteStack;
    TrajectorySequence wiggle;
    //endregion

    ArmController armController;
    DcMotorEx hangMotor;

    //region Intake Objects
    DcMotorEx intakeMotor;
    Servo intakeServo;

    public static double INTAKE_POWER = 1; //Power of Intake Motor
    public static double INTAKE_POSITION = 0; //Position of Intake Servo
    boolean intakePrepare = false;
    boolean intakeActive = false;
    boolean reverseIntake = false;

    public static double PIXEL_DETECTION_DISTANCE = 1.25; //Distance from color sensor to pixel for detection (CM)
    public double reverseTimer = 0; //timer for reversing intake
    public static double REVERSE_TIME = 1000; //How long to Reverse intake
    //endregion

    //region Vision Objects
    TeamPropDetection propDetection;
    String screenSector;
    int targetTagId;
    int targetTagIdWhite;
    int pixelsDropped = 0;
    AprilTagProcessor aprilTagDetector;
    VisionPortal visionPortal;
    AprilTagHomer aprilTagHomer;

    RevColorSensorV3 colorSensor1;
    RevColorSensorV3 colorSensor2;
    //endregion

    //region RR static coordinates

    //region red board spike locations
    Pose2d spikeLocation;

    Pose2d spikeLeft;
    Pose2d spikeCenter;
    Pose2d spikeRight;
    Pose2d spikeMiddle;
    //endregion

    Pose2d beforeTrussCord = new Pose2d(-36, 8, Math.toRadians(180));
    Pose2d afterTrussCord = new Pose2d(30, 8, Math.toRadians(180));
    Pose2d whiteStackCord = new Pose2d(-56, 11, Math.toRadians(180));


    Pose2d startingDrivePose = new Pose2d(10, 62, Math.toRadians(90));;

    Pose2d startingDrivePoseBoard = new Pose2d(10, 62, Math.toRadians(90));
    Pose2d startingDrivePoseAway = new Pose2d(-35, 62, Math.toRadians(90));


    //TODO: CHECK THESE
    public static Pose2d centerBlueBoardCord = new Pose2d(35, 39, Math.toRadians(180));
    public static Pose2d leftBlueBoardCord = new Pose2d(35, 45, Math.toRadians(180));
    public static Pose2d rightBlueBoardCord = new Pose2d(35, 32, Math.toRadians(180 ));
    public static Pose2d blueBoardCord = new Pose2d(35, 38, Math.toRadians(180));
    public static Pose2d blueParkCord = new Pose2d(48, 38, Math.toRadians(180));

    //endregion

    enum autoState { //Stores Different Auto States
        START,
        TO_SPIKE_MARK,
        VISION_SWITCH,
        PRE_TRUSS,
        TO_WHITE,
        GRAB_WHITE,
        POST_TRUSS,
        TO_BOARD,
        HOME_TAG,
        PLACE_BOARD,
        POST_DROP,
        PARK,
        STOP
    }

    autoState queuedState = autoState.START;

    //region AutoConfiguration Objects
    Gamepad currentGamepad;
    Gamepad previousGamepad;
    AutoConfiguration autoConfiguration;
    //endregion

     boolean hasTwoPixel;

    @Override
    public void runOpMode() {

        //region Init Objects and Variables

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

        //region Rigging Init
        hangMotor = hardwareMap.get(DcMotorEx.class,"hangMotor");
        hangMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        hangMotor.setPower(1);
        hangMotor.setTargetPosition(0);
        hangMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        hangMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        hangMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //endregion


        //region Vision Init

        aprilTagDetector = AprilTagProcessor.easyCreateWithDefaults();

        aprilTagHomer = new AprilTagHomer(aprilTagDetector, drive);

        propDetection = new TeamPropDetection("blue");

        colorSensor1 = hardwareMap.get(RevColorSensorV3.class,"colorSensor1");
        colorSensor2 = hardwareMap.get(RevColorSensorV3.class, "colorSensor2");

        visionPortal = VisionPortal.easyCreateWithDefaults(hardwareMap.get(WebcamName.class, "webcam"), aprilTagDetector, propDetection);
        visionPortal.setProcessorEnabled(aprilTagDetector, false);

        //endregion

        currentGamepad = new Gamepad();
        previousGamepad = new Gamepad();
        currentGamepad.copy(gamepad1);
        previousGamepad.copy(gamepad1);

        autoConfiguration = new AutoConfiguration(telemetry, AutoConfiguration.AllianceColor.BLUE);

        autoConfiguration.processInput(currentGamepad,previousGamepad);
        //endregion

        while(opModeInInit()){
            //Loops for auto configuration UI
            autoConfiguration.processInput(currentGamepad, previousGamepad);

            previousGamepad.copy(currentGamepad);
            currentGamepad.copy(gamepad1);
        }

        //Sets starting position based on start position variable
        if(autoConfiguration.getStartPosition() == AutoConfiguration.StartPosition.BOARD){
            startingDrivePose = startingDrivePoseBoard;
        }
        else{
            startingDrivePose = startingDrivePoseAway;
        }

        drive.setPoseEstimate(startingDrivePose);

        headingHelper.setInitialHeading(Math.toDegrees(startingDrivePose.getHeading()));

        while (opModeIsActive()) {
            telemetry.addData("Next auto State", queuedState);

            switch (queuedState) {
                case START:
                    //Setts Spike Marks per starting position
                    if(autoConfiguration.getStartPosition() == AutoConfiguration.StartPosition.BOARD){
                        spikeLeft = new Pose2d(20.5,37, Math.toRadians(120));
                        spikeCenter = new Pose2d(12,31, Math.toRadians(90));
                        spikeRight = new Pose2d(2,38, Math.toRadians(45));
                    }else{
                        spikeRight = new Pose2d(-42,35, Math.toRadians(45));
                        spikeCenter = new Pose2d(-35,31.5, Math.toRadians(90));
                        spikeLeft = new Pose2d(-31,28, Math.toRadians(180));
                        spikeMiddle = new Pose2d(-34.5,38, Math.toRadians(90));
                    }


                    spikeLocation = spikeLeft;
                
                    //Obtains team prop location from propDetector
                    screenSector = propDetection.getScreenSector();
                    if (screenSector != null) {
                        if (screenSector.equals("L")) {
                            spikeLocation = spikeLeft;
                            blueBoardCord = leftBlueBoardCord;
                            targetTagId = 1;
                            targetTagIdWhite = 2;
                        } else if (screenSector.equals("C")) {
                            beforeTrussCord = new Pose2d(-36, 11, Math.toRadians(180));
                            afterTrussCord = new Pose2d(30, 11, Math.toRadians(180));
                            spikeLocation = spikeCenter;
                            blueBoardCord = centerBlueBoardCord;
                            targetTagId = 2;
                            targetTagIdWhite = 1;
                        }
                        else {
                            beforeTrussCord = new Pose2d(-36, 8.5, Math.toRadians(165));
                            afterTrussCord = new Pose2d(30, 8.5, Math.toRadians(180));
                            spikeLocation = spikeRight;
                            blueBoardCord = rightBlueBoardCord;
                            targetTagId = 3;
                            targetTagIdWhite = 1;
                        }
                    }
                    else {
                        spikeLocation = spikeRight;
                        blueBoardCord = rightBlueBoardCord;
                        targetTagId = 6;
                        targetTagIdWhite = 5;
                    }
                    //tag assignment based of starting position
                    if(autoConfiguration.getStartPosition() == AutoConfiguration.StartPosition.AWAY && autoConfiguration.isWhitePixels()){
                        aprilTagHomer.changeTarget(targetTagIdWhite);
                    }
                    else{
                        aprilTagHomer.changeTarget(targetTagId);
                    }

                    //Adds for
                    waitTimer = System.currentTimeMillis() + autoConfiguration.getDelay() * 1000L;
                    queuedState = autoState.TO_SPIKE_MARK;
                    break;
                case TO_SPIKE_MARK:
                    if (!drive.isBusy() && System.currentTimeMillis() > waitTimer) {
                        switch (autoConfiguration.getStartPosition()) { //Switch for Spike Mark Pathing Depending on Side of Board

                            //region Board Spike Mark Pathing
                            case BOARD:
                                if(!Objects.equals(screenSector, "R")) {
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
                                if(Objects.equals(screenSector, "L")) {
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
                                            .strafeLeft(16)
                                            .back(28)
                                            .build();
                                    drive.followTrajectorySequenceAsync(toSpikeMark);
                                }
                                else if (Objects.equals(screenSector, "R")) {
                                    toSpikeMark = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                                            .back(12)
                                            .lineToLinearHeading(spikeLocation)
                                            .forward(7)
                                            .lineToLinearHeading(spikeMiddle, SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL, Math.toRadians(160), DriveConstants.TRACK_WIDTH),
                                                    SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                                            .lineToLinearHeading(new Pose2d(beforeTrussCord.getX(), beforeTrussCord.getY(), Math.toRadians(90)))
                                            .build();
                                    drive.followTrajectorySequenceAsync(toSpikeMark);
                                }
                                break;
                            //endregion
                        }

                        //Stops Program if only Purple Pixel was selected
                        if (autoConfiguration.isPurplePixelOnly()) {
                            queuedState = autoState.STOP;
                        } else {
                            queuedState = autoState.VISION_SWITCH;
                        }
                    }
                    break;
                case VISION_SWITCH:
                    if (!drive.isBusy()) {

                        headingHelper.loopMethod();

                        //Switches CVision Pipeline from team prop to april tag detection
                        visionPortal.setProcessorEnabled(propDetection, false);
                        visionPortal.setProcessorEnabled(aprilTagDetector, true);

                        //If on side close to board goes to place pixel, If not goes to pre truss location
                        if(autoConfiguration.getStartPosition() == AutoConfiguration.StartPosition.BOARD)
                        queuedState = autoState.TO_BOARD;
                        else queuedState = autoState.PRE_TRUSS;
                    }
                    break;
                case PRE_TRUSS:
                    if(!drive.isBusy()){
                        headingHelper.loopMethod();

                        toPreTruss = drive.trajectoryBuilder(drive.getPoseEstimate())
                                .lineToLinearHeading(beforeTrussCord, SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL, Math.toRadians(160), DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                                .build();
                        drive.followTrajectoryAsync(toPreTruss);

                        //If white pixels is enabled and hasn't gone to stack, goes to pixel stack
                        //If not goes under truss
                        if (!hasTwoPixel && autoConfiguration.isWhitePixels()) {
                            intakeActive = true;
                            queuedState = autoState.TO_WHITE;
                            //aprilTagHomer.changeTarget(targetTagIdWhite);
                        }
                        else queuedState = autoState.POST_TRUSS;
                    }
                    break;
                case TO_WHITE:
                    if (!drive.isBusy()) {
                        headingHelper.loopMethod();
                        toWhiteStack = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                                .lineToLinearHeading(whiteStackCord,
                                        SampleMecanumDrive.getVelocityConstraint(15, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                )
                                .addDisplacementMarker(() -> {
                                    waitTimer = System.currentTimeMillis() + WHITE_INTAKE_TIME;
                                    intakeActive = true;

                                })
                                .build();
                        drive.followTrajectorySequenceAsync(toWhiteStack);
                        queuedState = autoState.GRAB_WHITE;
                    }
                    break;
                case GRAB_WHITE:
                    if(!drive.isBusy()) {
                        headingHelper.loopMethod();

                        //Checks if both sensors have detected white pixel
                        if ((colorSensor1.getDistance(DistanceUnit.CM) <= PIXEL_DETECTION_DISTANCE && colorSensor2.getDistance(DistanceUnit.CM) <= PIXEL_DETECTION_DISTANCE) || System.currentTimeMillis() > waitTimer) {
                            intakeActive = false;
                            reverseIntake = true;
                            hasTwoPixel = true;
                            reverseTimer = System.currentTimeMillis() + REVERSE_TIME;
                            queuedState = autoState.PRE_TRUSS;
                        }
                        else {
                            //wiggle on that jiggle
                            wiggle = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                                    .turn(Math.toRadians(15))
                                    .turn(Math.toRadians(-30))
                                    .build();
                            drive.followTrajectorySequenceAsync(wiggle);
                        }
                    }
                    break;
                case POST_TRUSS:
                    if(!drive.isBusy() && waitTimer < System.currentTimeMillis()){
                        headingHelper.loopMethod();
                        toPostTruss = drive.trajectoryBuilder(drive.getPoseEstimate())
                                .addDisplacementMarker(() -> {
                                    if(armController.getCurrentArmState() == ArmController.ArmState.OUTTAKE_READY) {
                                        armController.switchArmState();
                                        armController.setSlideHeight(-10);
                                    }
                                })
                                .lineToLinearHeading(afterTrussCord)
                                .build();
                        drive.followTrajectoryAsync(toPostTruss);
                        if(!hasTwoPixel && autoConfiguration.isWhitePixels())
                            queuedState = autoState.PRE_TRUSS;
                        else
                            queuedState = autoState.TO_BOARD;
                    }
                    break;
                case TO_BOARD:
                    if (!drive.isBusy()) {
                        headingHelper.loopMethod();
                        intakeMotor.setPower(0);
                        toRedBoard = drive.trajectoryBuilder(drive.getPoseEstimate())
                                .lineToLinearHeading(blueBoardCord)
                                .build();
                        armController.switchArmState();
                        if (autoConfiguration.getStartPosition() == AutoConfiguration.StartPosition.AWAY &&
                                autoConfiguration.getAllianceYellow()){
                            armController.setStage(1);
                        }
                        drive.followTrajectoryAsync(toRedBoard);
                        queuedState = autoState.HOME_TAG;
                    }
                    break;
                case HOME_TAG:
                    if (!drive.isBusy()) {
                        aprilTagHomer.processRobotPosition();
                        aprilTagHomer.updateDrive();
                        waitTimer = System.currentTimeMillis() + APRIL_HOMER_LIMIT;
                        queuedState = autoState.PLACE_BOARD;
                    }
                    break;
                case PLACE_BOARD:

                    aprilTagHomer.processRobotPosition();
                    //Waits until board is in range or the wait timer is up to place pixel on board
                    if ((aprilTagHomer.inRange() || System.currentTimeMillis() > waitTimer) && !drive.isBusy()) {
                        armController.changeStage(1);

                        armController.startOuttake();
                        armController.startOuttake();
                        waitTimer = System.currentTimeMillis() + BOARD_OUTTAKE_TIME;

                        autoCycleCount ++;
                        queuedState = autoState.POST_DROP;
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
                case POST_DROP:
                    if(!drive.isBusy() && waitTimer < System.currentTimeMillis()){
                        if(hasTwoPixel){
                            //aprilTagHomer.changeTarget(targetTagIdWhite);
                            toRedBoard = drive.trajectoryBuilder(drive.getPoseEstimate())
                                    .lineToLinearHeading(blueBoardCord)
                                    .addDisplacementMarker(() -> {
                                        armController.changeStage(-1);
                                    })
                                    .build();
                            drive.followTrajectoryAsync(toRedBoard);
                            pixelsDropped ++;
                            //update tag assignment after first pixel is dropped
                            if (autoConfiguration.getStartPosition() == AutoConfiguration.StartPosition.AWAY){
                                if(pixelsDropped == 1) {
                                    aprilTagHomer.changeTarget(targetTagId);
                                }
                                else{
                                    aprilTagHomer.changeTarget(targetTagIdWhite);
                                }
                            }
                            if (autoConfiguration.getStartPosition() == AutoConfiguration.StartPosition.BOARD){
                                if(pixelsDropped >= 1) {
                                    aprilTagHomer.changeTarget(targetTagIdWhite);
                                }
                            }
                            queuedState = autoState.HOME_TAG;
                            hasTwoPixel = false;
                        }
                        else if(autoCycleCount <= autoConfiguration.getCycleCount() && autoConfiguration.isWhitePixels())
                            queuedState = autoState.POST_TRUSS;
                        else queuedState = autoState.PARK;
                    }
                    break;
                case PARK:
                    if (!drive.isBusy() && System.currentTimeMillis() > waitTimer) {
                        headingHelper.loopMethod();


                        //Trajectory to Park Pos
                        if(autoConfiguration.getParkSide() == AutoConfiguration.ParkSide.SIDE){
                            blueParkCord = new Pose2d(48, 60, Math.toRadians(180));
                        }else{
                            blueParkCord = new Pose2d(46, 10, Math.toRadians(180));
                        }

                        blueBoardPark = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                                .forward(10)
                                .addDisplacementMarker(() -> {
                                    armController.switchArmState();
                                })
                                .lineToLinearHeading(blueParkCord)
                                .build();
                        drive.followTrajectorySequenceAsync(blueBoardPark);
                        waitTimer = System.currentTimeMillis() + PARK_TIME;
                        queuedState = autoState.STOP;
                    }
                    break;
                case STOP:
                    if (!drive.isBusy()) {
                    }
            }

            if(intakeActive){
                intakePrepare = false;
                intakeMotor.setPower(INTAKE_POWER);
                intakeServo.setPosition(0);
                armController.setOuttakePower(-1);

            }
            //Checks if reverse is on and timer is still on
            else if (reverseIntake && reverseTimer > System.currentTimeMillis()){
                intakeServo.setPosition(0);
                intakeMotor.setPower(-INTAKE_POWER);
                armController.setOuttakePower(0);
            }
            else if (intakePrepare){
                intakeServo.setPosition(0);
                armController.setOuttakePower(0);
                intakeMotor.setPower(0);
            }
            else{
                intakeMotor.setPower(0);
                intakeServo.setPosition(1);
            }

            telemetry.update();

            drive.update();

            armController.updateArm();

            //Auto error threshold kill
            Pose2d lastError = drive.getLastError();

            if(Math.abs(lastError.getX()) > ERROR_THRESH || Math.abs(lastError.getY()) > ERROR_THRESH){
                requestOpModeStop();
            }
        }
    }
}
