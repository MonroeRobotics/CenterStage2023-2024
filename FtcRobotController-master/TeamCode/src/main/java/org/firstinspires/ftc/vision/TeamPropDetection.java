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

public class TeamPropDetection implements VisionProcessor {

    String alliance;

    public TeamPropDetection(String alliance) {
        this.alliance = alliance;
    }


    //Creates the upper and lower range for the accepted HSV values for color
    public Scalar lowHSV = new Scalar(0,0,0);
    public Scalar highHSV = new Scalar(179,255,255);
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


        if(alliance.equals("red")){
            //pink range
            lowHSV = new Scalar(168,60,60);
            highHSV = new Scalar(178,250,250);
            //red range
            //lowHSV = new Scalar(0,60,60);
            //highHSV = new Scalar(6,250,250);
        }else if(alliance.equals("blue")){
            //blue range
            lowHSV = new Scalar(104,40,40);
            highHSV = new Scalar(110,240,240);
        }

        //Returns Output Mat "thresh" that only contains pixels that are within low and high boundaries (lowHSV, highHSV)

        Core.inRange(input, lowHSV, highHSV, input);

        Imgproc.morphologyEx(input, input, MORPH_OPEN, Imgproc.getStructuringElement(MORPH_RECT, new Size(3, 3)));



        Rect leftScreen = new Rect(0, height/2, width/4, height/2);

        Rect centerScreen = new Rect(width/4, height/2, width/2, height/2);

        Rect rightScreen = new Rect((width/4)*3, height/2, width/4, height/2);


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

        return null; // No context object
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {
        // Not useful either
    }

}