package org.firstinspires.ftc.teamcode.legacy;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.util.ArmController;
import org.firstinspires.ftc.vision.AprilTagHomer;
import org.firstinspires.ftc.vision.TeamPropDetection;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.Objects;

@Autonomous(name = "Super Red Board Auto", group = "Super")
@Config
@Disabled
public class SuperAutoRedBoard extends OpMode {

    //region Dashboard Variable Declarations

    //region Auto Timer

    public static double SPIKE_OUTTAKE_TIME = 1000; //Time Spike Pixel Outtakes In auto
    public static double BOARD_OUTTAKE_TIME = 1000;//Time Board Pixel Outtakes in auto
    public static double PARK_TIME = 2000; //Time to go to park pos
    public static double APRIL_HOMER_LIMIT = 3000; //Failsafe for if apriltag homer has issues

    double waitTimer;


    //endregion

    public static double SPIKE_OUTTAKE_POWER = -0.3; //Stores the power of the reversed intake for spike pixel drop

    public static double CAMERA_EXPOSURE = 12;
    public static int CAMERA_GAIN = 255;
    //endregion

    SampleMecanumDrive drive;

    //region Trajectory Declarations
    Trajectory toSpikeMark;
    Trajectory toRedBoard;
    Trajectory redBoardPark1;
    Trajectory redBoardPark2;
    Trajectory grabWhite;
    Trajectory dropWhite;
    //endregion

    ArmController armController;

    //region Intake Objects
    DcMotorEx intakeMotor;
    Servo intakeServo;

    //endregion

    //region Vision Objects
    TeamPropDetection propDetection = new TeamPropDetection("red");
    String screenSector;
    int targetTagId;
    int whiteTargetTagId;
    AprilTagProcessor aprilTagDetector;
    VisionPortal visionPortal;
    AprilTagHomer aprilTagHomer;
    //endregion

    //region RR static coordinates

    //region red board spike locations
    Pose2d spikeLocation;

    Pose2d spikeLeft = new Pose2d(4,-40, Math.toRadians(315));
    Vector2d spikeLeftSpline = new Vector2d(11,-32);
    Pose2d spikeCenter = new Pose2d(12,-34.5, Math.toRadians(270));
    Pose2d spikeRight = new Pose2d(19.75,-37, Math.toRadians(240));
    //endregion

    public static Pose2d STARTING_DRIVE_POS = new Pose2d(10, -62, Math.toRadians(270));

    //y was previously -35
    public static Pose2d centerRedBoardCord = new Pose2d(35, -36, Math.toRadians(180));
    public static Pose2d rightRedBoardCord = new Pose2d(35, -40, Math.toRadians(180));
    public static Pose2d leftRedBoardCord = new Pose2d(35, -32, Math.toRadians(180));
    public static Pose2d redBoardCord = new Pose2d(35, -38, Math.toRadians(180));
    public static Pose2d redParkCord = new Pose2d(48, -64, Math.toRadians(180));
    public static Pose2d beforeTrussCord = new Pose2d(-36, -12, Math.toRadians(180));
    public static Pose2d afterTrussCord = new Pose2d(12, -12, Math.toRadians(180));
    public static Pose2d whiteStackCord = new Pose2d(-56, -11, Math.toRadians(180));

    enum autoState {
        START,
        TO_SPIKE_MARK,
        OUTTAKE_SPIKE,
        TO_BOARD,
        HOME_TAG,
        PLACE_BOARD,
        GRAB_WHITE,
        HOME_WHITE,
        PLACE_WHITE,
        PARK,
        STOP
    }

    autoState queuedState = autoState.START;

    @Override
    public void init() {

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        drive = new SampleMecanumDrive(hardwareMap);

        drive.setPoseEstimate(STARTING_DRIVE_POS);

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
                        redBoardCord = leftRedBoardCord;
                        targetTagId = 4;
                        whiteTargetTagId = 6;
                    } else if (screenSector.equals("C")) {
                        spikeLocation = spikeCenter;
                        redBoardCord = centerRedBoardCord;
                        targetTagId = 5;
                        whiteTargetTagId = 6;
                    } else {
                        spikeLocation = spikeRight;
                        redBoardCord = rightRedBoardCord;
                        targetTagId = 6;
                        whiteTargetTagId = 4;
                    }

                    queuedState = autoState.TO_SPIKE_MARK;
                }
                break;
            case TO_SPIKE_MARK:
                if(!drive.isBusy() && !Objects.equals(screenSector, "L")) {
                    toSpikeMark = drive.trajectoryBuilder(drive.getPoseEstimate())
                            .lineToLinearHeading(spikeLocation)
                            .forward(12)
                            .build();
                    drive.followTrajectoryAsync(toSpikeMark);
                    queuedState = autoState.OUTTAKE_SPIKE;
                }
                else if (!drive.isBusy()) {
                    toSpikeMark = drive.trajectoryBuilder(drive.getPoseEstimate())
                            .back(12)
                            .lineToLinearHeading(spikeLeft)
                            .forward(12)
                            .build();
                    drive.followTrajectoryAsync(toSpikeMark);
                    queuedState = autoState.OUTTAKE_SPIKE;
                }
                break;
            case OUTTAKE_SPIKE:
                if(!drive.isBusy()){
                    visionPortal.setProcessorEnabled(propDetection, false);
                    visionPortal.setProcessorEnabled(aprilTagDetector, true);
                    /*ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
                    if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
                        exposureControl.setMode(ExposureControl.Mode.Manual);
                    }
                    exposureControl.setExposure((long)CAMERA_EXPOSURE, TimeUnit.MILLISECONDS);

                    // Set Gain.
                    GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
                    gainControl.setGain(CAMERA_GAIN);*/
                    queuedState = autoState.TO_BOARD;
                }
                break;
            case TO_BOARD:
                if(!drive.isBusy() && System.currentTimeMillis() >= waitTimer){
                    intakeMotor.setPower(0);
                    toRedBoard = drive.trajectoryBuilder(toSpikeMark.end())
                            .lineToLinearHeading(redBoardCord)
                            .build();
                    armController.switchArmState();
                    drive.followTrajectoryAsync(toRedBoard);
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
                    armController.startOuttake();
                    waitTimer = System.currentTimeMillis() + BOARD_OUTTAKE_TIME;
                    queuedState = autoState.GRAB_WHITE;
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
            case GRAB_WHITE:
                if(!drive.isBusy()){
                    grabWhite = drive.trajectoryBuilder(drive.getPoseEstimate())
                            .lineToLinearHeading(afterTrussCord)
                            .lineToLinearHeading(beforeTrussCord)
                            .lineToLinearHeading(whiteStackCord)
                            //need more advanced grab
                            .build();
                    dropWhite = drive.trajectoryBuilder(drive.getPoseEstimate())
                            .lineToLinearHeading(beforeTrussCord)
                            .lineToLinearHeading(afterTrussCord)
                            .lineToLinearHeading(redBoardCord)
                            .build();
                    drive.followTrajectoryAsync(grabWhite);
                    armController.setOuttakePower(1);
                    waitTimer = System.currentTimeMillis() + BOARD_OUTTAKE_TIME;
                    if(System.currentTimeMillis() > waitTimer){
                        armController.setOuttakePower(0);
                        drive.followTrajectoryAsync(dropWhite);
                        queuedState = autoState.HOME_WHITE;
                    }
                }
            case HOME_WHITE:
                if (!drive.isBusy()){
                    aprilTagHomer.changeTarget(whiteTargetTagId);
                    aprilTagHomer.updateDrive();
                    waitTimer = System.currentTimeMillis() + APRIL_HOMER_LIMIT;
                    queuedState = autoState.PLACE_WHITE;
                }
            case PLACE_WHITE:
                if(aprilTagHomer.inRange() || System.currentTimeMillis() > waitTimer){
                    armController.startOuttake();
                    waitTimer = System.currentTimeMillis() + BOARD_OUTTAKE_TIME;
                    queuedState = autoState.GRAB_WHITE;
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
                    //Trajectory to Park Pos
                    redBoardPark2 = drive.trajectoryBuilder(drive.getPoseEstimate())
                            .lineToLinearHeading(redParkCord)
                            .build();
                    redBoardPark1 = drive.trajectoryBuilder(drive.getPoseEstimate())
                            .forward(5)
                            .addDisplacementMarker(() -> {
                                armController.switchArmState();
                                armController.setSlideHeight(-10);
                                drive.followTrajectoryAsync(redBoardPark2);
                            })
                            .build();
                    //Start Following Trajectory
                    drive.followTrajectoryAsync(redBoardPark1);
                    //Put slide and arm back to intake position


                    waitTimer = System.currentTimeMillis() + PARK_TIME;
                    queuedState = autoState.STOP;
                }
                break;
            case STOP:
                if(!drive.isBusy()){
                }
        }



        telemetry.update();

        drive.update();

        armController.updateArm();
    }
}
