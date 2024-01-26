package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class AutoConfiguration {

    Telemetry telemetry;

    public enum ParkSide{
        MIDDLE,
        SIDE
    }
    public enum AllianceColor{
        RED,
        BLUE
    }

    public enum AdjVariables{
        DELAY,
        WHITE_PIXELS,
        PARKSIDE
    }

    boolean whitePixels;
    int delay;

    ParkSide parkSide;
    AllianceColor allianceColor;
    AdjVariables currentVariable = AdjVariables.DELAY;

    AdjVariables[] adjVariables = AdjVariables.values();

    int currVIndex = 0;


    public AutoConfiguration(Telemetry telemetry, AllianceColor allianceColor){
        this.telemetry = telemetry;
        this.allianceColor = allianceColor;
        parkSide = ParkSide.SIDE;
        delay = 0;
        whitePixels = false;
    }
    public AutoConfiguration(Telemetry telemetry, AllianceColor allianceColor, ParkSide parkSide, int delay, boolean whitePixels){
        this.telemetry = telemetry;
        this.allianceColor = allianceColor;
        this.parkSide = parkSide;
        this.delay = delay;
        this.whitePixels = whitePixels;
    }

    public boolean isWhitePixels() {
        return whitePixels;
    }

    public int getDelay() {
        return delay;
    }

    public ParkSide getParkSide() {
        return parkSide;
    }

    public void processInput(Gamepad currentGamepad, Gamepad previousGamepad){
        if(currentGamepad.dpad_down && !previousGamepad.dpad_down && currVIndex < adjVariables.length){
            currVIndex += 1;
            currentVariable = adjVariables[currVIndex];
        }
        else if(currentGamepad.dpad_up && !previousGamepad.dpad_up && currVIndex > 0){
            currVIndex -= 1;
            currentVariable = adjVariables[currVIndex];
        }
        else if(currentGamepad.dpad_right && !previousGamepad.dpad_right) {
            switch (currentVariable) {
                case DELAY:
                    if(delay < 30){
                        delay += 1;
                    }
                    break;
                case WHITE_PIXELS:
                    whitePixels = !whitePixels;
                    break;
                case PARKSIDE:
                    if(parkSide == ParkSide.SIDE){
                        parkSide = ParkSide.MIDDLE;
                    }
                    else{
                        parkSide = ParkSide.SIDE;
                    }
                    break;
            }
        }
        else if(currentGamepad.dpad_left && !previousGamepad.dpad_left) {
            switch (currentVariable) {
                case DELAY:
                    if(delay > 0){
                        delay -= 1;
                    }
                    break;
                case WHITE_PIXELS:
                    whitePixels = !whitePixels;
                    break;
                case PARKSIDE:
                    if(parkSide == ParkSide.SIDE){
                        parkSide = ParkSide.MIDDLE;
                    }
                    else{
                        parkSide = ParkSide.SIDE;
                    }
                    break;
            }
        }

        telemetry.addData("Current Color", allianceColor);
        telemetry.addData(((currentVariable == AdjVariables.DELAY ? "*" : "") + "Delay"), delay);
        telemetry.addData(((currentVariable == AdjVariables.WHITE_PIXELS ? "*" : "") + "White Pixel"), whitePixels);
        telemetry.addData(((currentVariable == AdjVariables.PARKSIDE ? "*" : "") + "ParkSide"), parkSide);
        telemetry.update();
    }
}