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

public class TeamPropDetection implements VisionProcessor {

    //String alliance;
    //Telemetry telemetry;

    public TeamPropDetection(Telemetry telemetry) {
        //, String alliance
        //this.alliance = alliance;
        //this.telemetry = telemetry;
    }


    //Creates the upper and lower range for the accepted HSV values for color
    public Scalar lowHSV = new Scalar(170,60,60);
    public Scalar highHSV = new Scalar(176,250,250);
    Mat cropL = new Mat();
    Mat cropC = new Mat();
    Mat cropR = new Mat();
    int width = 640;
    int height = 480;
    String screenSector;
    public String getScreenSector(){
        return screenSector;
    }



    @Override
    public void init(int width, int height, CameraCalibration calibration) {
        // Not useful in this case, but we do need to implement it either way
        width = 640;
        height = 480;
    }

    @Override
    public Object processFrame(Mat input, long captureTimeNanos) {

        //changes Mat input from RGB to HSV and saves to Mat HSV
        Imgproc.cvtColor(input, input, Imgproc.COLOR_RGB2HSV);

        /*
        if(alliance.equals("red")){
             //pink range
             lowHSV = new Scalar(170,60,60);
             highHSV = new Scalar(176,250,250);
        }else if(alliance.equals("blue")){
             //purple range
             lowHSV = new Scalar(130,40,40);
             highHSV = new Scalar(136,240,240);
        }
         */

        //Returns Output Mat "thresh" that only contains pixels that are within low and high boundaries (lowHSV, highHSV)

        Core.inRange(input, lowHSV, highHSV, input);

        Imgproc.morphologyEx(input, input, MORPH_OPEN, Imgproc.getStructuringElement(MORPH_RECT, new Size(3, 3)));



        Rect leftScreen = new Rect(0, 0, width/4, height);

        Rect centerScreen = new Rect(width/4, 0, width/2, height);

        Rect rightScreen = new Rect((width/4)*3, 0, width/4, height);


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

        //telemetry.addData("[Zone]", screenSector);
        //telemetry.update();

        return null; // No context object
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {
        // Not useful either
    }

}