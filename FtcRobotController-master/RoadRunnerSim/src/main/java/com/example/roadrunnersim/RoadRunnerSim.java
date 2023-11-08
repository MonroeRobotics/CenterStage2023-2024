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

        Vector2d boardCord = new Vector2d(48, -35);
        Vector2d parkCord = new Vector2d(48, -60);

        // Declare our first bot
        RoadRunnerBotEntity firstBot = new DefaultBotBuilder(meepMeep)
                .setDimensions(14,16)
                // We set this bot to be blue
                .setColorScheme(new ColorSchemeBlueDark())
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(10, -62, Math.toRadians(90)))
                                .lineToLinearHeading(new Pose2d(32.75,-30, Math.toRadians(180)))
                                .lineToConstantHeading(boardCord)
                                .lineToConstantHeading(parkCord)
                                .build()
                );

        RoadRunnerBotEntity secondBot = new DefaultBotBuilder(meepMeep)
                .setDimensions(14,16)
                // We set this bot to be blue
                .setColorScheme(new ColorSchemeBlueDark())
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(10, -62, Math.toRadians(90)))
                                .lineToLinearHeading(new Pose2d(20,-24, Math.toRadians(180)))
                                .lineToConstantHeading(boardCord)
                                .lineToConstantHeading(parkCord)
                                .build()
                );
        RoadRunnerBotEntity thirdBot = new DefaultBotBuilder(meepMeep)
                .setDimensions(14,16)
                // We set this bot to be blue
                .setColorScheme(new ColorSchemeBlueDark())
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(10, -62, Math.toRadians(90)))
                                .lineToLinearHeading(new Pose2d(10,-30, Math.toRadians(180)))
                                .lineToConstantHeading(boardCord)
                                .lineToConstantHeading(parkCord)
                                .build()
                );
        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                // Add both of our declared bot entities
                .addEntity(firstBot)
                .addEntity(secondBot)
                .addEntity(thirdBot)
                .start();
    }
}