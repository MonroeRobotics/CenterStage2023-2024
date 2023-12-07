package org.firstinspires.ftc.teamcode.testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@TeleOp
@Config
public class LiftTuner extends OpMode {

    public static int SLIDE_HEIGHT = 20;
    public static double SLIDE_POWER = 0.5;
    public static double SLIDE_MAX_VELO = 2000;


    /*public static double Pv = 1;
    public static double Iv = 0;
    public static double Dv = 0;
    public static double Fv = 10;
    public static double Pp = 1;
*/





    DcMotorEx rightLinear;
    DcMotorEx leftLinear;

    //endregion


    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        //region Hardware Map


        leftLinear = hardwareMap.get(DcMotorEx.class ,"leftLinear");
        rightLinear = hardwareMap.get(DcMotorEx.class, "rightLinear");

        //endregion

        //region Motor Settings
        leftLinear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightLinear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftLinear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightLinear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftLinear.setTargetPosition(SLIDE_HEIGHT);
        rightLinear.setTargetPosition(SLIDE_HEIGHT);

        leftLinear.setPower(SLIDE_POWER);
        rightLinear.setPower(SLIDE_POWER);

        /*leftLinear.setPositionPIDFCoefficients(Pp);
        rightLinear.setPositionPIDFCoefficients(Pp);

        leftLinear.setVelocityPIDFCoefficients(Pv, Iv, Dv, Fv);
        rightLinear.setVelocityPIDFCoefficients(Pv, Iv, Dv, Fv);*/

        leftLinear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightLinear.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftLinear.setVelocity(SLIDE_MAX_VELO);
        rightLinear.setVelocity(SLIDE_MAX_VELO);
        //endregion


    }
    @Override
    public void loop() {
        leftLinear.setTargetPosition(SLIDE_HEIGHT);
        rightLinear.setTargetPosition(SLIDE_HEIGHT);

        telemetry.addData("Slide Target Height", SLIDE_HEIGHT);
        telemetry.addData("Right Slide Height", rightLinear.getCurrentPosition());
        telemetry.addData("left Slide Height", leftLinear.getCurrentPosition());
        telemetry.addData("Right Slide Volt", rightLinear.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("left Slide Volt", leftLinear.getCurrent(CurrentUnit.AMPS));

        telemetry.update();
    }



    @Override
    public void stop() {
    }
}
