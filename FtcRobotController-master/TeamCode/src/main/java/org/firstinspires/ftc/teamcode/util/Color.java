package org.firstinspires.ftc.teamcode.util;

public class Color {
    int[] rbg;

    public Color(int r, int g, int b){
        rbg = new int[]{r, g, b};
    }

    public Color(int[] rgb){
        this.rbg = rgb;
    }

    public int[] getRbg() {
        return rbg;
    }

    public void setRbg(int[] rbg) {
        this.rbg = rbg;
    }

    public int getError (Color color){
        int[] otherRGB = color.getRbg();

        int errorValue = 0;

        for(int i = 0; i < rbg.length; i++){
            errorValue += Math.abs(rbg[i] - otherRGB[i]);
        }

        return errorValue;
    }

}
