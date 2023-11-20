package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@TeleOp
public class AxisLocking extends OpMode {

    SampleMecanumDrive drive;

    //sensor on back-left robot
    DistanceSensor leftDistance;

    //sensor on back-right robot
    DistanceSensor rightDistance;

    //error for rightDist sensor, changable in dashboard
    public static double rightDistanceError = 0;

    //From 0<x<1, align factor
    double alignFactor = .1;

    //boolean if axisLock mode active
    boolean axisLocked = false;

    //enter in cm
    double targetDistance = 10;

    //for clicking reason
    Gamepad currentGamepad1;

    Gamepad previousGamepad1;



    @Override
    public void init() {
        //intialize hardware
        drive = new SampleMecanumDrive(hardwareMap);
        leftDistance = hardwareMap.get(DistanceSensor.class, "leftDistanceSensor");
        rightDistance = hardwareMap.get(DistanceSensor.class, "rightDistanceSensor");

        //turn off velocity control for reasons
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        currentGamepad1 = new Gamepad();
        previousGamepad1 = new Gamepad();
    }

    @Override
    public void loop() {

        //get base movement inputs
        double x = gamepad1.left_stick_x;
        double y = -gamepad1.left_stick_y;
        double heading = gamepad1.right_stick_x;

        //if gamepad1 x is clicked change axisLocked to other val
        //to make it not freak out
        if(currentGamepad1.cross && !previousGamepad1.cross){
            axisLocked = !axisLocked;
        }

        //if axis locked is true, auto align then move to targetDistance
        if(axisLocked){

            //subtracts right sensor distance from left sensor distance (right error in there too)
            heading = (rightDistance.getDistance(DistanceUnit.CM) - rightDistanceError)
                                    - leftDistance.getDistance(DistanceUnit.CM);
            //mult power by alignFactor
            heading *= alignFactor;

            //make it target distance once heading = 0 and aligned
            if(heading == 0){
                y = (targetDistance - (rightDistance.getDistance(DistanceUnit.CM) - rightDistanceError))
                        * alignFactor;
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
    }
}
