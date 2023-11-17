package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@TeleOp
public class AxisLocking extends OpMode {

    SampleMecanumDrive drive;

    DistanceSensor leftDistanceSensor;
    DistanceSensor rightDistanceSensor;

    //this gotta be from 0<x<1
    double alignHeadingFactor = .2;
    boolean axisLocked = false;
    boolean xNotPressed = true;

    @Override
    public void init() {
        //intialize the things
        drive = new SampleMecanumDrive(hardwareMap);
        leftDistanceSensor = hardwareMap.get(DistanceSensor.class, "leftDistanceSensor");
        rightDistanceSensor = hardwareMap.get(DistanceSensor.class, "rightDistanceSensor");

        //turn off velocity control for reasons
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    @Override
    public void loop() {

        //get them at base so I can mess with them if needed
        double x = gamepad1.left_stick_x
        double y = -gamepad1.left_stick_y;
        double heading = gamepad1.right_stick_x;

        //if gamepad1 x is clicked change axisLocked to other val
        if(gamepad1.x && xNotPressed){
            xNotPressed = false;
            axisLocked = !axisLocked;
        }
        if(!gamepad1.x){
            xNotPressed = true;
        }

        //if axis locked is true, auto align then lock y and heading changes
        if(axisLocked){

            //subtracts right sensor distance from left sensor distance
            heading = rightDistanceSensor.getDistance(DistanceUnit.CM)
                                    - leftDistanceSensor.getDistance(DistanceUnit.CM);

            //if diff pos, turn right, if neg, turn left, if zero, its already zero
            //mult power by alignHeadingFactor
            if(heading > 0){
                heading = 1 * alignHeadingFactor;
            } else if(heading < 0){
                heading = -1 * alignHeadingFactor;
            }



            //stop y and heading change
            y = 0;
        }


        // Pass in the rotated input + right stick value for rotation
        drive.setWeightedDrivePower(
                new Pose2d(
                        x,
                        y,
                        heading
                )
        );

        // Update everything. Odometry. Etc.
        drive.update();

        // Print pose to telemetry
        telemetry.addData("x", poseEstimate.getX());
        telemetry.addData("y", poseEstimate.getY());
        telemetry.addData("heading", poseEstimate.getHeading());
        telemetry.update();
    }
}
