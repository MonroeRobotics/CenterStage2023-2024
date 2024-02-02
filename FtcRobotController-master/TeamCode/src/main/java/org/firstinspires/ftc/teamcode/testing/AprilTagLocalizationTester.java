package org.firstinspires.ftc.teamcode.testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.vision.AprilTagHomer;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

@Autonomous(name="AprilTag Localization Tester", group = "Testing")
@Config
public class AprilTagLocalizationTester extends OpMode {
    AprilTagProcessor aprilTag;
    VisionPortal visionPortal;
    SampleMecanumDrive drive;
    AprilTagHomer tagHomer;
    AprilTagPoseFtc currentTagPose;

    public static int targetTag = 6;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        drive = new SampleMecanumDrive(hardwareMap);
        aprilTag = AprilTagProcessor.easyCreateWithDefaults();
        visionPortal = VisionPortal.easyCreateWithDefaults(hardwareMap.get(WebcamName.class, "webcam"), aprilTag);
        tagHomer = new AprilTagHomer(aprilTag, drive);
        tagHomer.changeTarget(targetTag);
    }

    @Override
    public void loop() {
        if(tagHomer.getCurrentTagPose() != null){
            currentTagPose = tagHomer.getCurrentTagPose();
        }

        tagHomer.changeTarget(targetTag);
        tagHomer.updateTag();


        telemetry.addLine("April Tag Data:");
        if(currentTagPose != null) {
            telemetry.addData("Tag X:", currentTagPose.x);
            telemetry.addData("Tag Y:", currentTagPose.y);
            telemetry.addData("Tag Yaw:", currentTagPose.yaw);
            telemetry.update();
        }
        else{
            telemetry.addLine("No Tag Detected");
        }

         tagHomer.processRobotPosition();
        drive.update();
        telemetry.update();
    }
}
