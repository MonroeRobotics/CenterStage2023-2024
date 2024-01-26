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
        START_POSITION,
        DELAY,
        PURPLEE_PIXEL_ONLY,
        WHITE_PIXELS,
        PARK_SIDE
    }

    public enum StartPosition{
        BOARD,
        AWAY
    }
    boolean whitePixels;
    boolean purplePixelOnly;

    int delay;

    ParkSide parkSide;
    StartPosition startPosition;
    AllianceColor allianceColor;
    AdjVariables currentVariable = AdjVariables.DELAY;

    AdjVariables[] adjVariables = AdjVariables.values();

    int currVIndex = 0;


    public AutoConfiguration(Telemetry telemetry, AllianceColor allianceColor){
        this.telemetry = telemetry;
        this.allianceColor = allianceColor;
        parkSide = ParkSide.SIDE;
        startPosition = StartPosition.BOARD;
        delay = 0;
        whitePixels = false;
        purplePixelOnly = false;
    }
    public AutoConfiguration(Telemetry telemetry, AllianceColor allianceColor, ParkSide parkSide, StartPosition startPosition, int delay, boolean whitePixels, boolean purplePixelOnly){
        this.telemetry = telemetry;
        this.allianceColor = allianceColor;
        this.parkSide = parkSide;
        this.startPosition = startPosition;
        this.delay = delay;
        this.whitePixels = whitePixels;
        this.purplePixelOnly = purplePixelOnly;
    }

    public boolean isWhitePixels() {
        return whitePixels;
    }

    public boolean isPurplePixelOnly(){
        return purplePixelOnly;
    }

    public int getDelay() {
        return delay;
    }

    public ParkSide getParkSide() {
        return parkSide;
    }

    public StartPosition getStartPosition(){
        return startPosition;
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
                case START_POSITION:
                    if(startPosition == StartPosition.BOARD){
                        startPosition = StartPosition.AWAY;
                    }
                    else{
                        startPosition = StartPosition.BOARD;
                    }
                    break;
                case DELAY:
                    if(delay < 30){
                        delay += 1;
                    }
                    break;
                case PURPLEE_PIXEL_ONLY:
                    purplePixelOnly = !purplePixelOnly;
                    break;
                case WHITE_PIXELS:
                    whitePixels = !whitePixels;
                    break;
                case PARK_SIDE:
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
                case START_POSITION:
                    if(startPosition == StartPosition.BOARD){
                        startPosition = StartPosition.AWAY;
                    }
                    else{
                        startPosition = StartPosition.BOARD;
                    }
                    break;
                case DELAY:
                    if(delay > 0){
                        delay -= 1;
                    }
                    break;
                case PURPLEE_PIXEL_ONLY:
                    purplePixelOnly = !purplePixelOnly;
                    break;
                case WHITE_PIXELS:
                    whitePixels = !whitePixels;
                    break;
                case PARK_SIDE:
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
        telemetry.addData(((currentVariable == AdjVariables.START_POSITION ? "*" : "") + "Start Position"), startPosition);
        telemetry.addData(((currentVariable == AdjVariables.DELAY ? "*" : "") + "Delay"), delay);
        telemetry.addData(((currentVariable == AdjVariables.PURPLEE_PIXEL_ONLY ? "*" : "") + "Purple Pixel Only"), purplePixelOnly);
        telemetry.addData(((currentVariable == AdjVariables.WHITE_PIXELS ? "*" : "") + "White Pixel"), whitePixels);
        telemetry.addData(((currentVariable == AdjVariables.PARK_SIDE ? "*" : "") + "Park Side"), parkSide);
        telemetry.update();
    }
}