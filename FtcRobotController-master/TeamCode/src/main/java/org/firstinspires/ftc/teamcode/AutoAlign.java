package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

@Autonomous(name="Auto Align")
public class AutoAlign extends OpMode {
    OpenCvWebcam webcam;
    int cameraMonitorViewId;

    double lBounding;
    double rBounding;

    int leftTarget = 330;
    int rightTarget = 440;
    double baseSpeedVert = 0.01;
    double baseSpeedHorz = 0.01;
    double motorVertical;
    double motorHorizontal;
    double multiplier = 0.25;
    int aOffset = 100;
    SampleMecanumDrive drive;
    @Override
    public void init() {
        cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        WebcamName webcamName = hardwareMap.get(WebcamName.class, "webcam");
        webcam = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                //sets webcam Computer Vision Pipeline to examplePipeline
                webcam.setPipeline(new AutoAlign.pipeDetect());
                webcam.startStreaming(640, 360, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });

        drive = new SampleMecanumDrive(hardwareMap);

    @Override
    public void loop() {

    }
}

    public class pipeDetect extends OpenCvPipeline {
        
    }
    }
