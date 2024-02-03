package org.firstinspires.ftc.teamcode.testing;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.vision.TeamPropAutoProcessor;
import org.firstinspires.ftc.vision.VisionPortal;

@Autonomous(name="Team Prop Auto Dect. Tuner", group = "Testing")
@Config
public class TeamPropDetectionAutoTuner extends OpMode {

    TeamPropAutoProcessor propDetection;
    VisionPortal visionPortal;

    Gamepad currentGamepad;
    Gamepad previousGamepad;


    @Override
    public void init() {

        propDetection = new TeamPropAutoProcessor();

        visionPortal = VisionPortal.easyCreateWithDefaults(hardwareMap.get(WebcamName.class, "webcam"), propDetection);
        //endregion

        currentGamepad = new Gamepad();
        previousGamepad = new Gamepad();
        currentGamepad.copy(gamepad1);
        previousGamepad.copy(gamepad1);
    }

    @Override
    public void loop() {
        telemetry.addLine("Press X when Cube is in middle");


        if(currentGamepad.cross && !previousGamepad.cross) {
            propDetection.getRanges();

        }

        telemetry.addData("Values", propDetection.getCurrentValues());

        telemetry.addData("lowHSV", propDetection.getLowHSV());
        telemetry.addData("highHSV", propDetection.getHighHSV());
        telemetry.update();

        previousGamepad.copy(currentGamepad);
        currentGamepad.copy(gamepad1);
    }
}
