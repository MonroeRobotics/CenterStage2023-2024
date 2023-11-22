package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.vision.AprilTagHomer;
import org.firstinspires.ftc.vision.TeamPropDetection;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

@Autonomous(name = "Test Auto Just Drive", group = "Main")
public class AutoProgramReadJustTrajectory extends LinearOpMode {
    SampleMecanumDrive drive;

    //region trajectory declarations
    Trajectory redBoardCenter;
    Trajectory redBoardPlacement;
    Trajectory redBoardPark;
    //endregion

    Trajectory universalTrajectory;
    Trajectory universalTrajectory2;

    //region red board spike locations
    Pose2d spikeLocation;
    Pose2d spikeOne = new Pose2d(10,-30, Math.toRadians(180));
    Pose2d spikeTwo = new Pose2d(20,-24, Math.toRadians(180));
    Pose2d spikeThree = new Pose2d(13,-30, Math.toRadians(0));
    //endregion

    double waitTime;
    double currentTime;

    WebcamName webcam;
    String screenSector;
    String colorDetectString = "";
    String colorDetected = "";

    //region importing vision stuff
    TeamPropDetection propDetection;

    AprilTagProcessor aprilTagDetector;
    VisionPortal visionPortal;

    AprilTagHomer aprilTagHomer;
    //endregion

    //region static coordinates
    Pose2d redBoardCord = new Pose2d(48, -35, Math.toRadians(180));
    Pose2d redParkCord = new Pose2d(48, -60, Math.toRadians(180));
    Pose2d blueBoardCord = new Pose2d(48, 35, Math.toRadians(180));
    Pose2d blueParkCord = new Pose2d(48, 60, Math.toRadians(180));

    Pose2d redBoardCenterSpikeMark = new Pose2d(13, -30, Math.toRadians(0));
    Pose2d redBoardCenterSpikeMarkStart = new Pose2d(10,62, Math.toRadians(90));
    //endregion
    @Override
    public void runOpMode() throws InterruptedException {

        //get webcam stuff here

        drive = new SampleMecanumDrive(hardwareMap);

        drive.setPoseEstimate(new Pose2d(10, -62, Math.toRadians(270)));

        //Identify spike marker location

        aprilTagDetector = AprilTagProcessor.easyCreateWithDefaults();

        aprilTagHomer = new AprilTagHomer(aprilTagDetector, drive);

        propDetection = new TeamPropDetection(telemetry);

        visionPortal = VisionPortal.easyCreateWithDefaults(hardwareMap.get(WebcamName.class, "webcam"), aprilTagDetector, propDetection);
        visionPortal.setProcessorEnabled(aprilTagDetector, false);

        waitForStart();

        screenSector = propDetection.getScreenSector();

        if (screenSector == "L"){
            spikeLocation = spikeOne;
        }
        else if (screenSector == "C"){
            spikeLocation = spikeTwo;
        }
        else{
            spikeLocation = spikeThree;
        }

        universalTrajectory = drive.trajectoryBuilder(drive.getPoseEstimate())
                .lineToLinearHeading(spikeLocation)

//                .addDisplacementMarker(() ->{
                      //place purple pixel on spike line
//                })
                .build();


        universalTrajectory2 = drive.trajectoryBuilder(universalTrajectory.end())
                .lineToLinearHeading(redBoardCord)
                .build();



        drive.followTrajectory(universalTrajectory);
        drive.followTrajectory(universalTrajectory2);
    }
}
