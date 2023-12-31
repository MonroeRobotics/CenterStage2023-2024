package org.firstinspires.ftc.vision;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

public class AprilTagHomerNoDrive {
    AprilTagProcessor aprilTag;
    AprilTagPoseFtc currentTagPose;
    int targetTagId = 1;
    double acptOffsetX = 0.1;
    double acptOffsetY = 0.1;
    double horizGain = 15;
    double vertGain = 15;

    double drivePowerX;
    double drivePowerY;


    public AprilTagHomerNoDrive(AprilTagProcessor aprilTag){
        this.aprilTag = aprilTag;

    }

    public AprilTagHomerNoDrive(AprilTagProcessor aprilTag, double acptOffsetX, double acptOffsetY, double horizGain, double vertGain) {
        this.aprilTag = aprilTag;
        this.acptOffsetX = acptOffsetX;
        this.acptOffsetY = acptOffsetY;
        this.horizGain = horizGain;
        this.vertGain = vertGain;
    }

    public double getDrivePowerX() {
        return drivePowerX;
    }

    public double getDrivePowerY() {
        return drivePowerY;
    }

    public AprilTagPoseFtc getCurrentTagPose() {
        return currentTagPose;
    }
    public void setGains (double horizGain, double vertGain){
        this.vertGain = vertGain;
        this.horizGain = horizGain;
    }

    public void setAcptOffsets (double acptOffsetX, double acptOffsetY){
        this.acptOffsetX = acptOffsetX;
        this.acptOffsetY = acptOffsetY;
    }

    public void updateDrive(){
        currentTagPose = updateCurrentTagPose();
        if (currentTagPose != null){

            // Y reversed because robot backwards
            //Set Max Possible power to 1
            drivePowerX = Math.min((Math.abs(currentTagPose.x) - acptOffsetX) / vertGain, 1);
            drivePowerY = -Math.min((Math.abs(currentTagPose.y) - acptOffsetY) / horizGain, 1);

            //Check Which Tag is On and reverse X power if necessary
            //CHECK THIS!!!!
            if (currentTagPose.x > 0){
                drivePowerX = -drivePowerX;
            }

            //drive.setDrivePower(new Pose2d(drivePowerX, drivePowerY, 0));
        }
        else {
            //drive.setDrivePower(new Pose2d(0,0, 0));
        }

    }

    public AprilTagPoseFtc updateCurrentTagPose() {

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
        if (currentTagPose != null) {
            if(Math.abs(currentTagPose.x) <= acptOffsetX && Math.abs(currentTagPose.y) <= acptOffsetY) {
                return true;
            }
        }
        return false;
    }
}
