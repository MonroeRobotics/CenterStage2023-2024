package org.firstinspires.ftc.teamcode.testing;

import static org.opencv.imgproc.Imgproc.MORPH_OPEN;
import static org.opencv.imgproc.Imgproc.MORPH_RECT;

import android.graphics.Canvas;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.AprilTagHomer;
import org.firstinspires.ftc.vision.TeamPropAutoProcessor;
import org.firstinspires.ftc.vision.TeamPropDetection;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

import dalvik.bytecode.Opcodes;

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


        if(currentGamepad.x && !previousGamepad.x) {
            telemetry.addData("lowHSV", propDetection.getLowHSV());
            telemetry.addData("highHSV", propDetection.getHighHSV());
            telemetry.update();

        }

        previousGamepad.copy(gamepad1);
        currentGamepad.copy(gamepad1);
    }
}
