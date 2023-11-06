package org.firstinspires.ftc.vision;


import static org.opencv.imgproc.Imgproc.MORPH_OPEN;
import static org.opencv.imgproc.Imgproc.MORPH_RECT;

import android.graphics.Canvas;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class pipelinesPlay implements VisionProcessor {

    Telemetry telemetry;

    public pipelinesPlay(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    Mat cropL = new Mat();
    Mat cropC = new Mat();
    Mat cropR = new Mat();

    String screenSector;

    @Override
    public void init(int width, int height, CameraCalibration calibration) {
        // Not useful in this case, but we do need to implement it either way
    }

    @Override
    public Object processFrame(Mat input, long captureTimeNanos) {

        //Imgproc.cvtColor(input, input, Imgproc.COLOR_RGB2GRAY);

        //changes Mat input from RGB to HSV and saves to Mat HSV
        Imgproc.cvtColor(input, input, Imgproc.COLOR_RGB2HSV);


        //Creates the upper and lower range for the accepted HSV values for color
        Scalar lowHSVM = new Scalar(154,40,40);
        Scalar highHSVM = new Scalar(179,240,240);

        //Returns Output Mat "thresh" that only contains pixels that are within low and high boundaries (lowHSV, highHSV)

        Core.inRange(input, lowHSVM, highHSVM, input);

        Imgproc.morphologyEx(input, input, MORPH_OPEN, Imgproc.getStructuringElement(MORPH_RECT, new Size(3, 3)));



        Rect leftScreen = new Rect(0, 0, 176/3, 144);

        Rect centerScreen = new Rect(176/3, 0, 176/3, 144);

        Rect rightScreen = new Rect((176/3)*2, 0, 176/3, 144);


        cropL = input.submat(leftScreen);
        cropC = input.submat(centerScreen);
        cropR = input.submat(rightScreen);

        Scalar avgValueL = Core.mean(cropL);
        Scalar avgValueC = Core.mean(cropC);
        Scalar avgValueR = Core.mean(cropR);

        if (avgValueL.val[0] > avgValueC.val[0] && avgValueL.val[0] > avgValueR.val[0]){
            screenSector = "L";
            Imgproc.rectangle(input, leftScreen, new Scalar(50,180,180));

        }

        else if (avgValueC.val[0] > avgValueL.val[0] && avgValueC.val[0] > avgValueR.val[0]){
            screenSector = "C";
            Imgproc.rectangle(input, centerScreen, new Scalar(50,180,180));
        }

        else{
            screenSector = "R";
            Imgproc.rectangle(input, rightScreen, new Scalar(50,180,180));
        }

        telemetry.addData("[Zone]", screenSector);
        telemetry.update();

        return null; // No context object
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {
        // Not useful either
    }

}