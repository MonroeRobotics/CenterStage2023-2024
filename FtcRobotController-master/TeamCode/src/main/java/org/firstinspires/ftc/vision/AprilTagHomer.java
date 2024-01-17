package org.firstinspires.ftc.vision;

import com.acmerobotics.roadrunner.drive.MecanumDrive;
import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

public class AprilTagHomer {
    AprilTagProcessor aprilTag;
    SampleMecanumDrive drive;
    AprilTagPoseFtc currentTagPose;
    int targetTagId = 6;
    double acptOffsetX = 1;
    double acptOffsetY = 2;
    double acptOffsetYaw = 0.5;
    double yawGain = 40;
    double cameraYawError = 0;
    double horizGain = 30;
    double vertGain = 50;
    double decelRate = 0.01;

    double drivePowerY = 0;
    double drivePowerX = 0;
    double drivePowerYaw = 0;


    public AprilTagHomer (AprilTagProcessor aprilTag, SampleMecanumDrive drive){
        this.aprilTag = aprilTag;
        this.drive = drive;

    }

    public AprilTagHomer(AprilTagProcessor aprilTag, SampleMecanumDrive drive, double acptOffsetX, double acptOffsetY, double horizGain, double vertGain, double decelRate, double yawGain, double acptOffsetYaw, double cameraYawError) {
        this.aprilTag = aprilTag;
        this.drive = drive;
        this.acptOffsetX = acptOffsetX;
        this.acptOffsetY = acptOffsetY;
        this.horizGain = horizGain;
        this.vertGain = vertGain;
        this.decelRate = decelRate;
        this.acptOffsetYaw = acptOffsetYaw;
        this.yawGain = yawGain;
        this.cameraYawError = cameraYawError;
    }

    public AprilTagPoseFtc getCurrentTagPose() {
        return currentTagPose;
    }
    public void setGains (double horizGain, double vertGain, double yawGain){
        this.vertGain = vertGain;
        this.horizGain = horizGain;
        this.yawGain = yawGain;
    }

    public void setAcptOffsets (double acptOffsetX, double acptOffsetY, double acptOffsetYaw){
        this.acptOffsetX = acptOffsetX;
        this.acptOffsetY = acptOffsetY;
        this.acptOffsetYaw = acptOffsetYaw;
    }

    public void updateTag(){
        currentTagPose = updateCurrentTagPose();
    }

    public void updateDrive(){
        updateTag();
        if (currentTagPose != null){

            // Y reversed because robot backwards
            //Set Max Possible power to 1
            drivePowerY = Math.min((Math.abs(currentTagPose.x)) / horizGain, 1);
            drivePowerX = -Math.min((Math.abs(currentTagPose.y)) / vertGain, .55);
            drivePowerYaw = -Math.min((Math.abs(currentTagPose.yaw - cameraYawError)) / yawGain , .05);
//          drivePowerX = 0;

            //Check Which Tag is On and reverse X power if necessary
            //CHECK THIS!!!!
            if (currentTagPose.x < 0){
                drivePowerY = -drivePowerY;
            }

            if (currentTagPose.y < 0){
                drivePowerX = -drivePowerX;
            }

            if (currentTagPose.yaw > 0){
                drivePowerYaw = -drivePowerYaw;
            }
        }
        else {
            if (drivePowerX >= decelRate){
                    drivePowerX -= decelRate;
            }
            else if(drivePowerX <= decelRate){
                drivePowerX += decelRate;
            }

            if (drivePowerY >= decelRate){
                drivePowerY -= decelRate;
            }
            else if(drivePowerY <= decelRate){
                drivePowerY += decelRate;
            }
            if (drivePowerYaw >= decelRate){
                drivePowerYaw -= decelRate;
            }
            else if(drivePowerYaw <= decelRate){
                drivePowerYaw += decelRate;
            }
        }

        drive.setDrivePower(new Pose2d(drivePowerX,drivePowerY, drivePowerYaw));
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
            if(Math.abs(currentTagPose.x) <= acptOffsetX && Math.abs(currentTagPose.y) <= acptOffsetY
                    && Math.abs(currentTagPose.yaw) <= acptOffsetYaw) {
                return true;
            }
        }
        return false;
    }
}
