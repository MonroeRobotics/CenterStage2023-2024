package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.util.AutoConfiguration;

@TeleOp
@Disabled
public class AutoConfigTester extends LinearOpMode {
    Gamepad currentGamepad;
    Gamepad previousGamepad;
    AutoConfiguration autoConfiguration;

    @Override
    public void runOpMode(){
        currentGamepad = new Gamepad();
        previousGamepad = new Gamepad();
        currentGamepad.copy(gamepad1);
        previousGamepad.copy(gamepad1);

        autoConfiguration = new AutoConfiguration(telemetry, AutoConfiguration.AllianceColor.RED);
        while(opModeInInit()){
            autoConfiguration.processInput(currentGamepad, previousGamepad);

            previousGamepad.copy(currentGamepad);
            currentGamepad.copy(gamepad1);
        }
    }
}
