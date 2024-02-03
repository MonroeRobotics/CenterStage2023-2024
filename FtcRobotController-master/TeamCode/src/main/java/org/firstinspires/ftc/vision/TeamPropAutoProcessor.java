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

    double[] currentValues;
    Scalar currentScalar;
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
        Rect centerScreen = new Rect(width/2 - width/8, height - height/4, width/4, height/4);


        cropC = input.submat(centerScreen);

        Imgproc.rectangle(input, centerScreen, new Scalar(50,180,180));

        return null; // No context object
    }

    public double[] getCurrentValues(){
        return currentValues;
    }

    public void getRanges(){

        double lowH = 255;
        double lowS = 255;
        double lowV = 255;

        double highH = 0;
        double highS = 0;
        double highV = 0;


        for(int i = 0; i < cropC.rows() - 1; i++){
            for(int k = 0; k < cropC.cols() - 1; k++){


                currentValues = cropC.get(i, k);

                if (currentValues != null && currentValues.length != 0){


                        if (currentValues[0] > highH) {
                            highH = currentValues[0];
                        }
                        if (currentValues[0] < lowH) {
                            lowH = currentValues[0];
                        }

                        if (currentValues[1] > highS) {
                            highS = currentValues[1];
                        }
                        if (currentValues[1] < lowS) {
                            lowS = currentValues[1];
                        }

                        if (currentValues[2] > highV) {
                            highV = currentValues[2];
                        }
                        if (currentValues[2] < lowV) {
                            lowV = currentValues[2];
                        }
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
