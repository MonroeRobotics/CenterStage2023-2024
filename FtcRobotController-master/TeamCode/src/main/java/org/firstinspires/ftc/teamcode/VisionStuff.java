package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.TeamPropDetection;
import org.firstinspires.ftc.vision.VisionPortal;

@Autonomous
public class VisionStuff extends OpMode {
    TeamPropDetection propDetector;
    VisionPortal visionPortal;
    String screenSector;
    @Override
    public void init() {
        propDetector = new TeamPropDetection(telemetry);
        visionPortal = VisionPortal.easyCreateWithDefaults(hardwareMap.get(WebcamName.class, "webcam"), propDetector);
    }

    @Override
    public void loop() {
        screenSector = propDetector.getScreenSector();
        telemetry.addData("Screen sector", screenSector);
    }
}
