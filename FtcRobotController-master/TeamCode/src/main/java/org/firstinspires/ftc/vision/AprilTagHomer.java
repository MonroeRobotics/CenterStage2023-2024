package org.firstinspires.ftc.vision;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@Config
public class AprilTagHomer {
    AprilTagProcessor aprilTag;
    SampleMecanumDrive drive;
    AprilTagDetection currentTag;
    AprilTagPoseFtc currentTagPose;

    public static double CAMERA_Y_OFF = 6;
    public static double CAMERA_X_OFF = -2;
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
        if(currentTag != null) {
            return currentTag.ftcPose;
        }
        else return null;
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
        currentTag = updateCurrentTag();
    }

    public void updateDrive(){
        updateTag();
        if (currentTag != null){
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

    public void processRobotPosition(){
        if(currentTag !=null) {
            Pose2d currPos = drive.getPoseEstimate();
            VectorF tagFieldPos = currentTag.metadata.fieldPosition;
            double newX = tagFieldPos.get(0) - currentTagPose.y - CAMERA_Y_OFF;
            double newY = tagFieldPos.get(1) + currentTagPose.x + CAMERA_X_OFF;
            drive.setPoseEstimate(new Pose2d(newX, newY, currPos.getHeading()));
        }
    }

    public AprilTagDetection updateCurrentTag() {

        List<AprilTagDetection> currentDetections = aprilTag.getDetections();

        // Step through the list of detections and check if matches target
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null && detection.id == targetTagId) {
                currentTagPose = detection.ftcPose;
                return detection;
            }
        }
        return null;
    }

    public void changeTarget (int newId){
        targetTagId = newId;
    }
    public boolean inRange (){
        if (currentTag != null) {
            if(Math.abs(currentTagPose.x) <= acptOffsetX && Math.abs(currentTagPose.y) <= acptOffsetY
                    && Math.abs(currentTagPose.yaw) <= acptOffsetYaw) {
                return true;
            }
        }
        return false;
    }
}
