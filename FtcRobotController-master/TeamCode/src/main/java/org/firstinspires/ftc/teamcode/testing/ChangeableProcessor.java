package org.firstinspires.ftc.teamcode.testing;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.TeamPropDetection;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

@Autonomous
@Config
@Disabled
public class ChangeableProcessor extends OpMode {
    TeamPropDetection propDetector;
    AprilTagProcessor aprilTagDetector;
    VisionPortal visionPortal;
    String screenSector;

    public static String alliance = "red";

    boolean cameraState = false;
    boolean notPressed = true;
    @Override
    public void init() {
        propDetector = new TeamPropDetection(alliance);
        aprilTagDetector = AprilTagProcessor.easyCreateWithDefaults();
        visionPortal = VisionPortal.easyCreateWithDefaults(hardwareMap.get(WebcamName.class, "webcam"), aprilTagDetector, propDetector);
        visionPortal.setProcessorEnabled(aprilTagDetector, false);
    }

    @Override
    public void loop() {
        screenSector = propDetector.getScreenSector();
        telemetry.addData("Screen sector", screenSector);
        if (gamepad1.a  && notPressed){
            notPressed = false;
            cameraState = !cameraState;
            if (cameraState == false){
                visionPortal.setProcessorEnabled(aprilTagDetector, false);
                visionPortal.setProcessorEnabled(propDetector, true);
            }
            else if (cameraState == true){
                visionPortal.setProcessorEnabled(aprilTagDetector, true);
                visionPortal.setProcessorEnabled(propDetector, false);
            }
        }
        if (!gamepad1.a){
            notPressed = true;
        }
    }
}
