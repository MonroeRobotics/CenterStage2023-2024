package com.example.roadrunnersim;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
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

        Pose2d redBoardCord = new Pose2d(43, -35, Math.toRadians(180));
        Pose2d redParkCord = new Pose2d(48, -60, Math.toRadians(180));
        Pose2d blueBoardCord = new Pose2d(48, 35, Math.toRadians(180));
        Pose2d blueParkCord = new Pose2d(48, 60, Math.toRadians(180));
        Pose2d STARTING_DRIVE_POS = new Pose2d(10, -62, Math.toRadians(270));

        // Declare our first bot
        Vector2d spikeLeftSpline = new Vector2d(11,-32);// Math.toRadians(180));
        Vector2d spikeCenterSpline = new Vector2d(20,-25.5);
        RoadRunnerBotEntity testBot = new DefaultBotBuilder(meepMeep)
                .setDimensions(15.25,18)
                .setColorScheme(new ColorSchemeRedDark())
                .setConstraints(60,60,Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(STARTING_DRIVE_POS)
                                .back(24)
                                //.strafeLeft(4)
                                //.splineTo(spikeCenterSpline, Math.toRadians(180))
                                .splineTo(spikeLeftSpline, Math.toRadians(0))
                                .build()
                );
        //region red board position
        //right
        RoadRunnerBotEntity firstBot = new DefaultBotBuilder(meepMeep)
                .setDimensions(14,16)
                .setColorScheme(new ColorSchemeRedDark())
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(STARTING_DRIVE_POS)
                                .lineToLinearHeading(new Pose2d(32,-30, Math.toRadians(180)))
                                .lineToLinearHeading(redBoardCord)
                                .lineToLinearHeading(redParkCord)
                                .build()
                );
        //center
        RoadRunnerBotEntity secondBot = new DefaultBotBuilder(meepMeep)
                .setDimensions(14,16)
                .setColorScheme(new ColorSchemeRedDark())
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(STARTING_DRIVE_POS)
                                .lineToLinearHeading(new Pose2d(20,-24, Math.toRadians(180)))
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
                                .lineToLinearHeading(new Pose2d(10,-30, Math.toRadians(180)))
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
                        drive.trajectorySequenceBuilder(new Pose2d(-35, -62, Math.toRadians(90)))
                                .lineToLinearHeading(new Pose2d(-44,-24, Math.toRadians(0)))
                                .lineToLinearHeading(new Pose2d(-44, -10, Math.toRadians(180)))
                                .lineToLinearHeading(new Pose2d(47, -10, Math.toRadians(180)))
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
                        drive.trajectorySequenceBuilder(new Pose2d(-35, -62, Math.toRadians(90)))
                                .lineToLinearHeading(new Pose2d(-33,-28, Math.toRadians(0)))
                                .lineToLinearHeading(new Pose2d(-30, -10, Math.toRadians(180)))
                                .lineToLinearHeading(new Pose2d(47, -10, Math.toRadians(180)))
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
                                .lineToLinearHeading(new Pose2d(-38,-30, Math.toRadians(180)))
                                .lineToLinearHeading(new Pose2d(-30, -10, Math.toRadians(180)))
                                .lineToLinearHeading(new Pose2d(47, -10, Math.toRadians(180)))
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
                .addEntity(testBot)
                //.addEntity(firstBot)
                //.addEntity(secondBot)
                //.addEntity(thirdBot)
                //.addEntity(fourBot)
                //.addEntity(fiveBot)
                //.addEntity(sixBot)
                //.addEntity(sevenBot)
                //.addEntity(eightBot)
                //.addEntity(nineBot)
                //.addEntity(tenBot)
                //.addEntity(elevenBot)
                //.addEntity(twelveBot)
                .start();
    }
}