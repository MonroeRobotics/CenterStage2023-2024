package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Disabled
@Autonomous(name = "Auto Program", group = "Main")
public class AutoProgram extends OpMode {
    SampleMecanumDrive drive;
    ChangeableProcessor changeableProcessor = new ChangeableProcessor();
    //trajectory declarations
    Trajectory redBoardCenter;
    Trajectory redBoardPlacement;
    Trajectory redBoardPark;

    Trajectory universalTrajectory;

    //slides
    DcMotorEx leftSlide;
    DcMotorEx rightSlide;
    Servo leftArmServo;
    Servo rightArmServo;

    Pose2d spikeLocation;
    Pose2d spikeOne = new Pose2d(10,-30, Math.toRadians(180));
    Pose2d spikeTwo = new Pose2d(20,-24, Math.toRadians(180));
    Pose2d spikeThree = new Pose2d(13,-30, Math.toRadians(0));


    //intake and placement pieces
    DcMotorEx intake;
    Servo uppy;
    Servo bucketServo;
    Servo bucketAngle;

    double waitTime;
    double currentTime;

    WebcamName webcam;

    String screenSector;
    String colorDetectString = "";
    String colorDetected = "";

    Pose2d redBoardCord = new Pose2d(48, -35, Math.toRadians(180));
    Pose2d redParkCord = new Pose2d(48, -60, Math.toRadians(180));
    Pose2d blueBoardCord = new Pose2d(48, 35, Math.toRadians(180));
    Pose2d blueParkCord = new Pose2d(48, 60, Math.toRadians(180));

    Pose2d redBoardCenterSpikeMark = new Pose2d(13, -30, Math.toRadians(0));
    Pose2d redBoardCenterSpikeMarkStart = new Pose2d(10,62, Math.toRadians(90));



    enum State {

    }

    State currentState;

    public String detectColor(){
        String detection = colorDetectString;

        return detection;
    }


    @Override
    public void init() {
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        uppy = hardwareMap.get(Servo.class, "uppy");
        bucketServo = hardwareMap.get(Servo.class, "bucketServo");
        bucketAngle = hardwareMap.get(Servo.class, "bucketAngle");

        leftSlide = hardwareMap.get(DcMotorEx.class, "leftSlide");
        rightSlide = hardwareMap.get(DcMotorEx.class, "rightSlide");

        leftArmServo = hardwareMap.get(Servo.class, "leftArmServo");
        rightArmServo = hardwareMap.get(Servo.class, "rightArmServo");

        DistanceSensor distanceSensor = hardwareMap.get(DistanceSensor.class, "distance");

        leftArmServo.setPosition(.72);
        rightArmServo.setPosition(.3);

        leftSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        rightSlide.setTargetPosition(10);
        leftSlide.setTargetPosition(10);
        rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightSlide.setPower(0.4);
        leftSlide.setPower(0.4);

        //get webcam stuff here

        drive = new SampleMecanumDrive(hardwareMap);

        drive.setPoseEstimate(new Pose2d(10, 62, Math.toRadians(90)));

        //Identify spike marker location

        if (screenSector == "L"){
            spikeLocation = spikeOne;
        }
        else if (screenSector == "C"){
            spikeLocation = spikeTwo;
        }
        else{
            spikeLocation = spikeThree;
        }

        universalTrajectory = drive.trajectoryBuilder(drive.getPoseEstimate())
            .lineToLinearHeading(spikeLocation)
                //place purple pixel on spike line
                /*.addDisplacementMarker(() -> {
                })*/
                .lineToLinearHeading(redBoardCord)
                //go to april tag indicated by spike marker location
                //.addDisplacementMarker()
                .lineToLinearHeading(redParkCord)
                .build();

        drive.followTrajectoryAsync(universalTrajectory);

    }

    @Override
    public void loop() {
        drive.update();
    }
}
