package org.firstinspires.ftc.teamcode.testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.util.Encoder;

@TeleOp
public class EncoderChecker extends OpMode {
    public Encoder rightOdo;
    public Encoder leftOdo;
    public Encoder backOdo;


    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        leftOdo = new Encoder(hardwareMap.get(DcMotorEx.class, "rightRear"));
        rightOdo = new Encoder(hardwareMap.get(DcMotorEx.class, "leftFront"));
        backOdo = new Encoder(hardwareMap.get(DcMotorEx.class, "rightFront"));
    }

    @Override
    public void loop() {
        telemetry.addData("rightOdo", rightOdo.getCurrentPosition());
        telemetry.addData("leftOdo", leftOdo.getCurrentPosition());
        telemetry.addData("backOdo", backOdo.getCurrentPosition());
        telemetry.update();
    }
}
