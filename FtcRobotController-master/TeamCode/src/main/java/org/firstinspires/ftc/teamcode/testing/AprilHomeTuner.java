package org.firstinspires.ftc.teamcode.testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.vision.AprilTagHomer;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

@Autonomous(name="AprilTag Homing Tuner", group = "Testing")
@Config
public class AprilHomeTuner extends OpMode {
    AprilTagProcessor aprilTag;
    VisionPortal visionPortal;
    SampleMecanumDrive drive;
    AprilTagHomer tagHomer;

    public static int targetTag = 6;
    public static double acptOffsetX = 1;
    public static double acptOffsetY = 7;
    public static double acptOffsetYaw = 1;
    public static double yawGain = 30;
    public static double cameraYawError = 0;
    public static double horizGain = 25;
    public static double vertGain = 60;

    public static double decelRate = 0.01;


    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        drive = new SampleMecanumDrive(hardwareMap);
        aprilTag = AprilTagProcessor.easyCreateWithDefaults();
        visionPortal = VisionPortal.easyCreateWithDefaults(hardwareMap.get(WebcamName.class, "webcam"), aprilTag);
        tagHomer = new AprilTagHomer(aprilTag, drive, acptOffsetX, acptOffsetY, horizGain, vertGain, decelRate, yawGain, acptOffsetYaw, cameraYawError);
        tagHomer.changeTarget(targetTag);
    }

    @Override
    public void loop() {
        tagHomer.setGains(horizGain, vertGain, yawGain);
        tagHomer.setAcptOffsets(acptOffsetX, acptOffsetY, acptOffsetYaw);

        AprilTagPoseFtc currentTagPose = tagHomer.getCurrentTagPose();

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

         if(!tagHomer.inRange()){
             tagHomer.updateDrive();
         }
         else{requestOpModeStop();}
        telemetry.update();
    }
}
