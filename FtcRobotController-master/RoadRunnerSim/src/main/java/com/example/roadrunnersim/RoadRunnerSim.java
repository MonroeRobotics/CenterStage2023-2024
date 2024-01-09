package com.example.roadrunnersim;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueDark;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedDark;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import java.awt.Image;
import java.io.File;
import java.io.IOException;
import java.nio.file.Path;
import java.nio.file.Paths;

import javax.imageio.ImageIO;

public class RoadRunnerSim {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        //region red board spike locations
        Pose2d spikeLocation;

        Pose2d spikeRight = new Pose2d(-10,-30, Math.toRadians(0));
        Vector2d spikeRightSpline = new Vector2d(-34,-29);
        Pose2d spikeCenter = new Pose2d(-48,-24, Math.toRadians(0));
        Pose2d spikeLeft = new Pose2d(-36,-29, Math.toRadians(180));
        //endregion

        //Pose2d redBoardCord = new Pose2d(35, -36, Math.toRadians(180));
        Pose2d blueBoardCord = new Pose2d(48, 35, Math.toRadians(180));
        Pose2d blueParkCord = new Pose2d(48, 60, Math.toRadians(180));
        Pose2d STARTING_DRIVE_POS = new Pose2d(10, -62, Math.toRadians(270));

        Pose2d centerRedBoardCord = new Pose2d(35, -36, Math.toRadians(180));
        Pose2d rightRedBoardCord = new Pose2d(35, -40, Math.toRadians(180));
        Pose2d leftRedBoardCord = new Pose2d(35, -32, Math.toRadians(180));
        Pose2d redBoardCord = new Pose2d(35, -38, Math.toRadians(180));
        Pose2d beforeTrussCord = new Pose2d(-48, -12, Math.toRadians(180));
        Pose2d afterTrussCord = new Pose2d(12, -12, Math.toRadians(180));
        Pose2d redParkCord = new Pose2d(48, -64, Math.toRadians(180));

        Vector2d redBoardSpline = new Vector2d(35, -36);

        // Declare our first bot
        Vector2d spikeLeftSpline = new Vector2d(11,-32);// Math.toRadians(180));

        Trajectory trussPath;
        Trajectory trussPath2;
        Trajectory trussPath3;

        //Pose2d beforeTrussCord = new Pose2d(-36, -10, Math.toRadians(0));
        //Pose2d afterTrussCord = new Pose2d(12, -10, Math.toRadians(0));
        //Pose2d spikeCenter = new Pose2d(-42,-24, Math.toRadians(0));
        //Vector2d spikeRightSpline = new Vector2d(-34,-29);
        // spikeLeft = new Pose2d(-36,-29, Math.toRadians(180));

        //Pose2d redParkCord = new Pose2d(-48, -64, Math.toRadians(0));

        RoadRunnerBotEntity testBot = new DefaultBotBuilder(meepMeep)
                .setDimensions(15.25,18)
                .setColorScheme(new ColorSchemeRedDark())
                .setConstraints(60,60,Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-36, -62, Math.toRadians(90)))
                                .lineToLinearHeading(spikeLeft)
                                .lineToLinearHeading(beforeTrussCord)
                                .lineToLinearHeading(afterTrussCord)
                                .lineToLinearHeading(redBoardCord)
                                .lineToLinearHeading(redParkCord)
                                .build()
                );
        //region red board position
        //right
        RoadRunnerBotEntity firstBot = new DefaultBotBuilder(meepMeep)
                .setDimensions(14,18)
                .setColorScheme(new ColorSchemeRedDark())
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(STARTING_DRIVE_POS)
                                .lineToLinearHeading(new Pose2d(19,-37, Math.toRadians(240)))
                                .forward(16)
                                .lineToLinearHeading(redBoardCord)
                                .lineToLinearHeading(redParkCord)
                                .build()
                );
        //center
        RoadRunnerBotEntity secondBot = new DefaultBotBuilder(meepMeep)
                .setDimensions(14,18)
                .setColorScheme(new ColorSchemeRedDark())
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(STARTING_DRIVE_POS)
                                .lineToLinearHeading(new Pose2d(12,-33, Math.toRadians(270)))
                                .forward(12)
                                .lineToLinearHeading(redBoardCord)
                                .lineToLinearHeading(redParkCord)
                                .build()
                );
        //left
        RoadRunnerBotEntity thirdBot = new DefaultBotBuilder(meepMeep)
                .setDimensions(14,16)
                // We set this bot to be blue
                .setColorScheme(new ColorSchemeRedDark())
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(STARTING_DRIVE_POS)
                                .lineToLinearHeading(new Pose2d(10,-30, Math.toRadians(0)))
                                .forward(12)
                                .lineToLinearHeading(redBoardCord)
                                .lineToLinearHeading(redParkCord)
                                .build()
                );
        // endregion

        //region red away position
        //center
        RoadRunnerBotEntity fourBot = new DefaultBotBuilder(meepMeep)
                .setDimensions(14,16)
                // We set this bot to be blue
                .setColorScheme(new ColorSchemeRedDark())
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-35, -62, Math.toRadians(270)))
                                .lineToLinearHeading(new Pose2d(-35,-33, Math.toRadians(270))) //drop off
                                .forward(12) //dropping off action
                                .strafeRight(12)
                                .lineToLinearHeading(beforeTrussCord)
                                .lineToLinearHeading(afterTrussCord)
                                .lineToLinearHeading(redBoardCord)
                                .lineToLinearHeading(redParkCord)
                                .build()
                );

        //right
        RoadRunnerBotEntity fiveBot = new DefaultBotBuilder(meepMeep)
                .setDimensions(14,16)
                // We set this bot to be blue
                .setColorScheme(new ColorSchemeRedDark())
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-35, -62, Math.toRadians(270)))
                                .lineToLinearHeading(new Pose2d(-33,-28, Math.toRadians(180))) //drop off cord
                                .forward(12) //dropping off action
                                .strafeRight(12)
                                .lineToLinearHeading(beforeTrussCord)
                                .lineToLinearHeading(afterTrussCord)
                                .lineToLinearHeading(redBoardCord)
                                .lineToLinearHeading(redParkCord)
                                .build()
                );
        //left
        RoadRunnerBotEntity sixBot = new DefaultBotBuilder(meepMeep)
                .setDimensions(14,16)
                // We set this bot to be blue
                .setColorScheme(new ColorSchemeRedDark())
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-35, -62, Math.toRadians(90)))
                                .lineToLinearHeading(new Pose2d(-38,-30, Math.toRadians(0)))
                                .forward(12) //dropping off action
                                .strafeRight(12)
                                .lineToLinearHeading(beforeTrussCord)
                                .lineToLinearHeading(afterTrussCord)
                                .lineToLinearHeading(redBoardCord)
                                .lineToLinearHeading(redParkCord)
                                .build()
                );
        //endregion

        //region blue away position
        //right
        RoadRunnerBotEntity sevenBot = new DefaultBotBuilder(meepMeep)
                .setDimensions(14,16)
                // We set this bot to be blue
                .setColorScheme(new ColorSchemeBlueDark())
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-35, 62, Math.toRadians(270)))
                                .lineToLinearHeading(new Pose2d(-38,30, Math.toRadians(180)))
                                .lineToLinearHeading(new Pose2d(-30, 10, Math.toRadians(180)))
                                .lineToLinearHeading(new Pose2d(47, 10, Math.toRadians(180)))
                                .lineToLinearHeading(blueBoardCord)
                                .lineToLinearHeading(blueParkCord)
                                .build()
                );
        //left
        RoadRunnerBotEntity eightBot = new DefaultBotBuilder(meepMeep)
                .setDimensions(14,16)
                // We set this bot to be blue
                .setColorScheme(new ColorSchemeBlueDark())
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-35, 62, Math.toRadians(90)))
                                .lineToLinearHeading(new Pose2d(-33,28, Math.toRadians(0)))
                                .lineToLinearHeading(new Pose2d(-30, 10, Math.toRadians(180)))
                                .lineToLinearHeading(new Pose2d(47, 10, Math.toRadians(180)))
                                .lineToLinearHeading(blueBoardCord)
                                .lineToLinearHeading(blueParkCord)
                                .build()
                );
        //center
        RoadRunnerBotEntity nineBot = new DefaultBotBuilder(meepMeep)
                .setDimensions(14,16)
                // We set this bot to be blue
                .setColorScheme(new ColorSchemeBlueDark())
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-35, 62, Math.toRadians(90)))
                                .lineToLinearHeading(new Pose2d(-44,24, Math.toRadians(0)))
                                .lineToLinearHeading(new Pose2d(-44, 10, Math.toRadians(180)))
                                .lineToLinearHeading(new Pose2d(47, 10, Math.toRadians(180)))
                                .lineToLinearHeading(blueBoardCord)
                                .lineToLinearHeading(blueParkCord)
                                .build()
                );
        //endregion

        //region blue board position
        //right
        RoadRunnerBotEntity tenBot = new DefaultBotBuilder(meepMeep)
                .setDimensions(14,16)
                // We set this bot to be blue
                .setColorScheme(new ColorSchemeBlueDark())
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(10, 62, Math.toRadians(90)))
                                .lineToLinearHeading(new Pose2d(10,30, Math.toRadians(180)))
                                .lineToLinearHeading(blueBoardCord)
                                .lineToLinearHeading(blueParkCord)
                                .build()
                );
        //center
        RoadRunnerBotEntity elevenBot = new DefaultBotBuilder(meepMeep)
                .setDimensions(14,16)
                // We set this bot to be blue
                .setColorScheme(new ColorSchemeBlueDark())
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(10, 62, Math.toRadians(90)))
                                .lineToLinearHeading(new Pose2d(20,24, Math.toRadians(180)))
                                .lineToLinearHeading(blueBoardCord)
                                .lineToLinearHeading(blueParkCord)
                                .build()
                );
        //left
        RoadRunnerBotEntity twelveBot = new DefaultBotBuilder(meepMeep)
                .setDimensions(14,16)
                // We set this bot to be blue
                .setColorScheme(new ColorSchemeBlueDark())
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(10, 62, Math.toRadians(90)))
                                .lineToLinearHeading(new Pose2d(13,30, Math.toRadians(0)))
                                .lineToLinearHeading(blueBoardCord)
                                .lineToLinearHeading(blueParkCord)
                                .build()
                );
        //endregion

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                // Add both of our declared bot entities
                //.addEntity(testBot)
                //.addEntity(firstBot)
                //.addEntity(secondBot)
                //.addEntity(thirdBot)
                .addEntity(fourBot)
                //.addEntity(fiveBot)
                .addEntity(sixBot)
                //.addEntity(sevenBot)
                //.addEntity(eightBot)
                //.addEntity(nineBot)
                //.addEntity(tenBot)
                //.addEntity(elevenBot)
                //.addEntity(twelveBot)
                .start();
    }
}