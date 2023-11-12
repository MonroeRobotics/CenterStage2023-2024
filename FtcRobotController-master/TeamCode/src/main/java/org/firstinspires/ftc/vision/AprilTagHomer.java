package org.firstinspires.ftc.vision;

import com.acmerobotics.roadrunner.drive.MecanumDrive;
import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

public class AprilTagHomer {
    AprilTagProcessor aprilTag;
    MecanumDrive drive;
    AprilTagPoseFtc currentTagPose;
    int targetTagId = 1;
    double acptOffsetX = 0.1;
    double acptOffsetY = 0.1;
    double horizGain = 15;
    double vertGain = 15;


    public AprilTagHomer (AprilTagProcessor aprilTag, MecanumDrive drive){
        this.aprilTag = aprilTag;
        this.drive = drive;

    }

    public AprilTagHomer(AprilTagProcessor aprilTag, MecanumDrive drive, double acptOffsetX, double acptOffsetY, double horizGain, double vertGain) {
        this.aprilTag = aprilTag;
        this.drive = drive;
        this.acptOffsetX = acptOffsetX;
        this.acptOffsetY = acptOffsetY;
        this.horizGain = horizGain;
        this.vertGain = vertGain;
    }

    public void updateDrive(){
        currentTagPose = getCurrentTagPose();
        if (currentTagPose != null){

            // Y reversed because robot backwards
            double drivePowerX = (Math.abs(currentTagPose.x) - acptOffsetX) / vertGain;
            double drivePowerY = -(Math.abs(currentTagPose.y) - acptOffsetY) / horizGain;

            //Check Which Tag is On and reverse power if necessary
            //CHECK THIS!!!!
            if (currentTagPose.x > 0){
                drivePowerX = -drivePowerX;
            }

            drive.setDrivePower(new Pose2d(drivePowerX,drivePowerY));
        }
        else {
            drive.setDrivePower(new Pose2d(0,0, 0));
        }

    }

    public AprilTagPoseFtc getCurrentTagPose() {

        List<AprilTagDetection> currentDetections = aprilTag.getDetections();

        // Step through the list of detections and check if matches target
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null && detection.id == targetTagId) {
                return detection.ftcPose;
            }
        }
        return null;
    }

    public void changeTarget (int newId){
        targetTagId = newId;
    }
    public boolean inRange (){
        if (Math.abs(currentTagPose.x) <= acptOffsetX && Math.abs(currentTagPose.y) <= acptOffsetY) {
            return true;
        }
        return false;
    }
}
