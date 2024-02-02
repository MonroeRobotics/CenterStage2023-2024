package org.firstinspires.ftc.teamcode.testing;

import static org.opencv.imgproc.Imgproc.MORPH_OPEN;
import static org.opencv.imgproc.Imgproc.MORPH_RECT;

import android.graphics.Canvas;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.TeamPropDetection;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

import dalvik.bytecode.Opcodes;

@Autonomous(name="Team Prop Auto Dect. Tuner", group = "Testing")
@Config
public class TeamPropDetectionAuto extends OpMode {


    @Override
    public void init() {

    }

    @Override
    public void loop() {

    }

    public class TeamPropAutoProcessor implements VisionProcessor{

    Telemetry telemetry;
    enum colorAlliance {
        RED,
        BLUE
    }

    public static colorAlliance currentColor = colorAlliance.BLUE;

    Scalar lowHSV;
    Scalar highHSV;
    Mat cropC = new Mat();
    int width = 640;
    int height = 480;

    @Override
    public void init(int width, int height, CameraCalibration calibration) {
        // Not useful in this case, but we do need to implement it either way
        width = 640;
        height = 480;
    }

    public Object processFrame(Mat input, long captureTimeNanos) {

        //changes Mat input from RGB to HSV and saves to Mat HSV
        Imgproc.cvtColor(input, input, Imgproc.COLOR_RGB2HSV);

        //creates center sqaure
        Rect centerScreen = new Rect(width/4, height/4, width/4, height/4);


        cropC = input.submat(centerScreen);




        return null; // No context object
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {
        // Not useful either
    }
}
}
