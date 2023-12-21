package org.firstinspires.ftc.teamcode.testing;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
@Config
public class AxonTest extends OpMode {

    public static double targetPosition = 0.4;  // Set your desired target position
    public static double kP = 0.1;  // Proportional constant
    public static double kI = 0; // Integral constant
    public static double kD = 0.05; // Derivative constant

    double integral = 0;
    double previousError = 0;

    Servo servo1;
    Servo servo2;
    AnalogInput servo1Input;

    AnalogInput servo2Input;

    Gamepad currentGamePad;
    Gamepad previousGamePad;
    @Override
    public void init() {
        servo1 = hardwareMap.get(Servo.class, "servo1");
        servo2 = hardwareMap.get(Servo.class, "servo2");

        servo1Input = hardwareMap.get(AnalogInput.class, "servo1Input");
        servo2Input = hardwareMap.get(AnalogInput.class, "servo2Input");

        servo1.setPosition(0);
        servo2.setPosition(1);
        currentGamePad = new Gamepad();
        previousGamePad = new Gamepad();
    }

    @Override
    public void loop() {

        /*if(currentGamePad.cross && !previousGamePad.cross){
            servo1.setPosition(0);
            servo2.setPosition(1);
        }
        if(currentGamePad.square && !previousGamePad.square){
            servo1.setPosition(1);
            servo2.setPosition(0);
        }


        previousGamePad.copy(currentGamePad);
        currentGamePad.copy(gamepad1);*/
        servo1.setPosition(servo1.getPosition() + servoPID());
        telemetry.addData("Servo 1 Reading", servo1Input.getVoltage());
        telemetry.addData("Servo 2 Reading", servo2Input.getVoltage());

        telemetry.update();


    }

    public double servoPID(){
        double currentPosition = servo1Input.getVoltage()/2.88;
        double error = targetPosition - currentPosition;

        double proportionalTerm = kP * error;

        // Calculate integral term (running sum of errors)
        // Note: This is a simple integral, more advanced techniques may be needed
        // such as the trapezoidal rule or anti-windup mechanisms.
        double integralTerm = kI * (integral + error);

        // Calculate derivative term (change in error)
        double derivativeTerm = kD * (error - previousError);

        // Calculate the output power
        double output = proportionalTerm + integralTerm + derivativeTerm;

        // Update previous error and integral for the next iteration
        previousError = error;
        integral += error;

        telemetry.addData("Target Position", targetPosition);
        telemetry.addData("Current Position", currentPosition);
        telemetry.addData("Output", output);

        return output;
    }
}
