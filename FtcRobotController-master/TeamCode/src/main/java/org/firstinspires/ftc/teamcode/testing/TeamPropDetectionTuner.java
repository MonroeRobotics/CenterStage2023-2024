package org.firstinspires.ftc.teamcode.testing;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.TeamPropDetection;
import org.firstinspires.ftc.vision.VisionPortal;
import org.opencv.core.Scalar;

@Autonomous(name="Team Prop Dect. Tuner", group = "Testing")
@Config
public class TeamPropDetectionTuner extends OpMode {

    enum colorAlliance {
        RED,
        BLUE
    }

    public static colorAlliance currentColor = colorAlliance.RED;

    public static Scalar lowHSVRed = new Scalar(168,60,60);
    public static Scalar highHSVRed = new Scalar(178,250,250);

    public static Scalar lowHSVBlue = new Scalar(115,40,40);
    public static Scalar highHSVBlue = new Scalar(125,240,240);

    Gamepad currentGamepad;
    Gamepad previousGamepad;

    TeamPropDetection propDetectorRed;
    TeamPropDetection propDetectorBlue;
    VisionPortal visionPortal;
    String screenSector;
    @Override
    public void init() {
        propDetectorBlue = new TeamPropDetection("blue");
        propDetectorRed = new TeamPropDetection("red");
        visionPortal = VisionPortal.easyCreateWithDefaults(hardwareMap.get(WebcamName.class, "webcam"), propDetectorRed, propDetectorBlue);
        visionPortal.setProcessorEnabled(propDetectorBlue, false);

        currentGamepad = new Gamepad();
        previousGamepad = new Gamepad();

        currentGamepad.copy(gamepad1);
        currentGamepad.copy(gamepad1);
    }

    @Override
    public void loop() {

        if (currentGamepad.x && !previousGamepad.x){
            if(currentColor == colorAlliance.BLUE){
                currentColor = colorAlliance.RED;
            }
            else {
                currentColor = colorAlliance.BLUE;
            }
        }

        switch (currentColor){
            case RED:
                if (!visionPortal.getProcessorEnabled(propDetectorRed)) {
                    visionPortal.setProcessorEnabled(propDetectorBlue, false);
                    visionPortal.setProcessorEnabled(propDetectorRed, true);
                }
                propDetectorRed.changeHSV(lowHSVRed, highHSVRed);
                screenSector = propDetectorRed.getScreenSector();
                telemetry.addData("Screen sector", screenSector);
                break;
            case BLUE:
                if (!visionPortal.getProcessorEnabled(propDetectorBlue)) {
                    visionPortal.setProcessorEnabled(propDetectorRed, false);
                    visionPortal.setProcessorEnabled(propDetectorBlue, true);
                }
                propDetectorBlue.changeHSV(lowHSVBlue,highHSVBlue);
                screenSector = propDetectorBlue.getScreenSector();
                telemetry.addData("Screen sector", screenSector);
                break;
        }

        previousGamepad.copy(currentGamepad);
        currentGamepad.copy(gamepad1);

        telemetry.update();

    }
}
