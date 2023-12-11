package org.firstinspires.ftc.teamcode.testing;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@TeleOp
@Config
@Disabled
public class AxisLocking extends OpMode {

    SampleMecanumDrive drive;

    //sensor on back-left robot
    DistanceSensor leftDistance;

    //sensor on back-right robot
    DistanceSensor rightDistance;

    //error for rightDist sensor, changable in dashboard
    public static double leftDistanceError = 0;

    double xError;
    double headingError;

    public static double headingTolerance = .3;
    public static double xTolerance = .3;


    //From 0<x<1, align factor
    public static double headingAlignFactor = 20;
    public static double xAlignFactor = 20;

    //boolean if axisLock mode active
    boolean axisLocked = false;

    //enter in cm
    public static double targetDistance = 16;

    //for clicking reason
    Gamepad currentGamepad1;

    Gamepad previousGamepad1;



    @Override
    public void init() {

        telemetry = new MultipleTelemetry(telemetry);
        //intialize hardware
        drive = new SampleMecanumDrive(hardwareMap);
        leftDistance = hardwareMap.get(DistanceSensor.class, "leftDistance");
        rightDistance = hardwareMap.get(DistanceSensor.class, "rightDistance");

        //turn off velocity control for reasons
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        currentGamepad1 = new Gamepad();
        previousGamepad1 = new Gamepad();
    }

    @Override
    public void loop() {

        //get base movement inputs
        double x = -gamepad1.left_stick_y;
        double y = -gamepad1.left_stick_x;
        double heading = -gamepad1.right_stick_x;

        //if gamepad1 x is clicked change axisLocked to other val
        //to make it not freak out
        if(currentGamepad1.cross && !previousGamepad1.cross){
            axisLocked = !axisLocked;
        }

        //if axis locked is true, auto align then move to targetDistance
        if(axisLocked){
            x = 0;

            headingError = (leftDistance.getDistance(DistanceUnit.CM) - leftDistanceError)
                    - rightDistance.getDistance(DistanceUnit.CM);

            if(Math.abs(headingError) >= headingTolerance) {
                //subtracts right sensor distance from left sensor distance (right error in there too)
                heading = headingError / headingAlignFactor;

            }else{
                heading = 0;


                //make it target distance
                xError = targetDistance - rightDistance.getDistance(DistanceUnit.CM);

                if(Math.abs(xError) >= xTolerance) {
                    //subtracts right sensor distance from left sensor distance (right error in there too)
                    x = xError / xAlignFactor;
                }

            }


        }


        // Pass in movement to setDrivePower
        drive.setWeightedDrivePower(
                new Pose2d(
                        x,
                        y,
                        heading
                )
        );

        // Update drive
        drive.update();

        previousGamepad1.copy(currentGamepad1);

        currentGamepad1.copy(gamepad1);

        telemetry.addData("Left Distance", leftDistance.getDistance(DistanceUnit.CM));
        telemetry.addData("Right Distance", rightDistance.getDistance(DistanceUnit.CM));

        telemetry.addData("X Error", xError);
        telemetry.addData("Heading Error", headingError);

        telemetry.update();
    }
}
