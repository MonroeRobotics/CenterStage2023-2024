package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

public class HeadingHelper {
    SampleMecanumDrive drive;
    HardwareMap hardwareMap;
    IMU imu;
    Telemetry telemetry;

    double initialHeading;

    public static double pollInterval = 2000;


    //USB Backward, Logo Left
    public static RevHubOrientationOnRobot.LogoFacingDirection LOGO_FACING_DIR =
            RevHubOrientationOnRobot.LogoFacingDirection.LEFT;
    public static RevHubOrientationOnRobot.UsbFacingDirection USB_FACING_DIR =
            RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD;

    double timer;

    public HeadingHelper(SampleMecanumDrive drive, HardwareMap hardwareMap, Telemetry telemetry) {
        this.drive = drive;
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;

        initialHeading = drive.getPoseEstimate().getHeading();

        timer = System.currentTimeMillis();

        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                LOGO_FACING_DIR, USB_FACING_DIR));
        imu.initialize(parameters);
        imu.resetYaw();
    }

    public void init(){

    }

    public void loopMethod(){
        YawPitchRollAngles yPR = getImuReading();
        telemetry.addData("Yaw", yPR.getYaw(AngleUnit.DEGREES));
        telemetry.addData("Adj. Yaw", getAdjustedYaw());
        telemetry.addData("Adj. Yaw Radians", Math.toRadians(getAdjustedYaw()));
        telemetry.addData("Pitch", yPR.getPitch(AngleUnit.DEGREES));
        telemetry.addData("Roll", yPR.getRoll(AngleUnit.DEGREES));
        telemetry.addData("Timer", timer);

        if (timer < System.currentTimeMillis()){
            Pose2d currPose = drive.getPoseEstimate();
            drive.setPoseEstimate(new Pose2d(currPose.getX(), currPose.getY(), Math.toRadians(getAdjustedYaw())));
            timer = System.currentTimeMillis() + pollInterval;
        }
    }

    public YawPitchRollAngles getImuReading(){
        return imu.getRobotYawPitchRollAngles();
    }

    public double getAdjustedYaw(){
        return getImuReading().getYaw(AngleUnit.DEGREES) + initialHeading;
    }

    public void setInitialHeading(double initialHeading){
        this.initialHeading = initialHeading;
    }

}
