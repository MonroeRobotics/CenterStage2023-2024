package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@TeleOp
public class AxisLocking extends OpMode {

    SampleMecanumDrive drive;

    boolean axisLocked = false;
    boolean xNotPressed = true;

    @Override
    public void init() {
        //intialize the MecanumDrive thingy
        drive = new SampleMecanumDrive(hardwareMap);

        //turn off velocity control for reasons
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //put in intial pos and heading from auto mode (magic stuff)
        drive.setPoseEstimate(new Pose2d(x, y, heading));
    }

    @Override
    public void loop() {
        // Read pose
        Pose2d poseEstimate = drive.getPoseEstimate();

        // Create a vector from the gamepad x/y inputs
        // Then, rotate that vector by the inverse of that heading
        Vector2d input = new Vector2d(
                -gamepad1.left_stick_y,
                -gamepad1.left_stick_x
        ).rotated(-poseEstimate.getHeading());

        //get them out before being put into setWDP so I can mess with them
        double x = input.getX();
        double y = input.getY();

        //if x is clicked change axisLocked to other val
        if(gamepad1.x && xNotPressed){
            xNotPressed = false;
            axisLocked = !axisLocked;
        }
        if(!gamepad1.x){
            xNotPressed = true;
        }

        //if axis locked is true, make y axis drive power 0
        if(axisLocked == true){
            y = 0;
        }

        // Pass in the rotated input + right stick value for rotation
        drive.setWeightedDrivePower(
                new Pose2d(
                        x,
                        y,
                        -gamepad1.right_stick_x
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
