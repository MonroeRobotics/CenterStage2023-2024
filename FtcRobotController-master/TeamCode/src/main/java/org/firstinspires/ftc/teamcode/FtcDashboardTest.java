package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@Autonomous
public class FtcDashboardTest extends LinearOpMode {
    public static double SERVO_POS = 0;
    Servo testServo;


    @Override
    public void runOpMode() {
        testServo = hardwareMap.get(Servo.class, "testServo");

        waitForStart();
        while (opModeIsActive()){
                testServo.setPosition(SERVO_POS);

        }
    }
}

