package org.firstinspires.ftc.vision;

import android.graphics.Canvas;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

public class TeamPropAutoProcessor implements VisionProcessor{

    Scalar lowHSV;
    Scalar highHSV;

    Telemetry telemetry;
    Mat cropC = new Mat();
    int width = 640;
    int height = 480;

    public Scalar getLowHSV() {
        return lowHSV;
    }

    public Scalar getHighHSV() {
        return highHSV;
    }

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

        Imgproc.rectangle(input, centerScreen, new Scalar(50,180,180));

        return null; // No context object
    }

    public void getRanges(){
        double lowH = 0;
        double lowS = 0;
        double lowV = 0;

        double highH = 0;
        double highS = 0;
        double highV = 0;

        for(int i = 0; i < cropC.height(); i++){
            for(int j = 0; j < cropC.width(); i++){
                double[] currentValue = cropC.get(i, j);
                if(currentValue[0] > highH){
                    currentValue[0] = highH;
                }
                else if (currentValue[0] < lowH){
                    currentValue[0] = lowH;
                }


                if(currentValue[1] > highS){
                    currentValue[1] = highS;
                }
                else if (currentValue[1] < lowS){
                    currentValue[1] = lowS;
                }


                if(currentValue[2] > highV){
                    currentValue[2] = highV;
                }
                else if (currentValue[2] < lowV){
                    currentValue[2] = lowV;
                }
            }
        }

        lowHSV = new Scalar(lowH, lowS, lowV);

        highHSV = new Scalar(highH, highS, highV);
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {
        // Not useful either
    }
}
