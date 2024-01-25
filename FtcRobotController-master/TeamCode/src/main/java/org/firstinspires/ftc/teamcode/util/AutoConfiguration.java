package org.firstinspires.ftc.teamcode.util;

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

    public enum currVari{
        delay
    }

    boolean whitePixels;
    int delay;

    ParkSide parkSide;
    AllianceColor allianceColor;
    currVari cV = currVari.delay;

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

    public void loopInit(){
        telemetry.addData(((cV == currVari.delay ? "*" : "") + "Delay"), delay);
    }
}
