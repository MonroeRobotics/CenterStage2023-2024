package org.firstinspires.ftc.teamcode.drive;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.ThreeTrackingWheelLocalizer;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.util.Encoder;

import java.util.Arrays;
import java.util.List;

/*
 * Sample tracking wheel localizer implementation assuming the standard configuration:
 *
 *    /--------------\
 *    |     ____     |
 *    |     ----     |
 *    | ||        || |
 *    | ||        || |
 *    |              |
 *    |              |
 *    \--------------/
 *
 */
@Config
public class StandardTrackingWheelLocalizer extends ThreeTrackingWheelLocalizer {
    public static double TICKS_PER_REV = 8192;
    public static double WHEEL_RADIUS = .6889764; // in
    public static double GEAR_RATIO = 1; // output (wheel) speed / input (encoder) speed

    public static double LATERAL_DISTANCE = 12.88703; // in; distance between the left and right wheels
    public static double FORWARD_OFFSET = 2.375; // in; offset of the lateral wheel

    private Encoder leftEncoder, rightEncoder, frontEncoder;

    public static double xMultiplier = 1.005238849351563;
    public static double yMultiplier = 1.0044;

    private List<Integer> lastEncPositions, lastEncVels;


    public StandardTrackingWheelLocalizer(HardwareMap hardwareMap, List<Integer> lastTrackingEncPositions, List<Integer> lastTrackingEncVels) {        super(Arrays.asList(
                new Pose2d(0, LATERAL_DISTANCE / 2, 0), // left
                new Pose2d(0, -LATERAL_DISTANCE / 2, 0), // right
                new Pose2d(FORWARD_OFFSET, 0, Math.toRadians(90)) // front
        ));

        lastEncPositions = lastTrackingEncPositions;
        lastEncVels = lastTrackingEncVels;

        leftEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "leftRear"));
        rightEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "rightRear"));
        frontEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "leftFront"));

        // TODO: reverse any encoders using Encoder.setDirection(Encoder.Direction.REVERSE)
        //Not Reversed as of 11/19/23
    }

    public static double encoderTicksToInches(double ticks) {
        return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
    }

    @NonNull
    @Override
    public List<Double> getWheelPositions() {
        int leftPos = leftEncoder.getCurrentPosition();
        int rightPos = rightEncoder.getCurrentPosition();
        int frontPos = frontEncoder.getCurrentPosition();

        lastEncPositions.clear();
        lastEncPositions.add(leftPos);
        lastEncPositions.add(rightPos);
        lastEncPositions.add(frontPos);

        return Arrays.asList(
                encoderTicksToInches(leftPos)*xMultiplier,
                encoderTicksToInches(rightPos)*xMultiplier,
                encoderTicksToInches(frontPos)*yMultiplier
        );
    }

    @NonNull
    @Override
    public List<Double> getWheelVelocities() {
        int leftVel = (int) leftEncoder.getCorrectedVelocity();
        int rightVel = (int) rightEncoder.getCorrectedVelocity();
        int frontVel = (int) frontEncoder.getCorrectedVelocity();

        lastEncVels.clear();
        lastEncVels.add(leftVel);
        lastEncVels.add(rightVel);
        lastEncVels.add(frontVel);

        return Arrays.asList(
                encoderTicksToInches(leftVel)*xMultiplier,
                encoderTicksToInches(rightVel)*xMultiplier,
                encoderTicksToInches(frontVel)*yMultiplier
        );
    }
}
