package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.vision.AprilTagHomer;
import org.firstinspires.ftc.vision.AprilTagHomerNoDrive;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

@Autonomous
@Config
public class AprilHomeTunerNoDrive extends OpMode {
    AprilTagProcessor aprilTag;
    VisionPortal visionPortal;
    AprilTagHomerNoDrive tagHomer;

    public static int targetTag = 1;
    public static double acptOffsetX = 0.1;
    public static double acptOffsetY = 0.1;
    public static double horizGain = 15;
    public static double vertGain = 15;


    @Override
    public void init() {
        aprilTag = AprilTagProcessor.easyCreateWithDefaults();
        visionPortal = VisionPortal.easyCreateWithDefaults(hardwareMap.get(WebcamName.class, "webcam"), aprilTag);
        tagHomer = new AprilTagHomerNoDrive(aprilTag, acptOffsetX, acptOffsetY, horizGain, vertGain);
        tagHomer.changeTarget(targetTag);
    }

    @Override
    public void loop() {
        tagHomer.setGains(horizGain, vertGain);
        tagHomer.setAcptOffsets(acptOffsetX, acptOffsetY);

        tagHomer.changeTarget(targetTag);

        if(!tagHomer.inRange()){
            tagHomer.updateDrive();
        }

        AprilTagPoseFtc currPos = tagHomer.getCurrentTagPose();
        if (currPos != null) {
            telemetry.addData("Tag X:",currPos.x);
            telemetry.addData("Tag Y:",currPos.y);
        }
        telemetry.addData("Motor X:", tagHomer.getDrivePowerX());
        telemetry.addData("Motor Y:", tagHomer.getDrivePowerY());
    }
}
