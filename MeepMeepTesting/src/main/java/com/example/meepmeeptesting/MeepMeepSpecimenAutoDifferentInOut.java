package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedLight;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepSpecimenAutoDifferentInOut {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(600);
        Vector2d firstReleasePos = new Vector2d(-4.5,-30);
        Vector2d secondReleasePos = new Vector2d(-1, -30);
        Vector2d thirdReleasePos = new Vector2d(2.5, -30);
        Vector2d fourthReleasePos = new Vector2d(5, -30);
        Vector2d retreatPos = new Vector2d(0,-35.5);
        Vector2d firstPiecePos = new Vector2d(48.5,-33.25);
        Vector2d secondPiecePos = new Vector2d(58.75,-33.25);
        Vector2d thirdPiecePos = new Vector2d(59, -25);
        Vector2d dropPos = new Vector2d(55, -59);
        Vector2d clipPos = new Vector2d(55, -61);
        Vector2d waitPos = new Vector2d(55, -47.5);
        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setColorScheme(new ColorSchemeRedLight())
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .setDimensions(17.5, 14.5)
                .setStartPose(new Pose2d(-34, -63, Math.toRadians(90)))
                .build();
        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(33, -63, Math.toRadians(90)))
                        .strafeTo(firstReleasePos)
                        .waitSeconds(0.5)
                        .strafeTo(retreatPos)
                        .waitSeconds(0.5)
                        .strafeTo(firstPiecePos)
                        .waitSeconds(0.5)
                        .strafeTo(dropPos)
                        .waitSeconds(0.5)
                        .strafeTo(secondPiecePos)
                        .waitSeconds(1)
                        .strafeToLinearHeading(dropPos, Math.toRadians(0))
                        .waitSeconds(0.5)
                        .strafeToLinearHeading(clipPos, Math.toRadians(90))
                        .waitSeconds(0.5)
                        .strafeToConstantHeading(secondReleasePos)
                        .waitSeconds(0.5)
                        .strafeTo(retreatPos)
                        .waitSeconds(0.5)
                        .strafeTo(new Vector2d(35,-33))
                        .strafeToSplineHeading(thirdPiecePos, Math.toRadians(0))
                        .waitSeconds(0.5)
                        .strafeTo(dropPos)
                        .waitSeconds(0.5)
                        .strafeToLinearHeading(clipPos, Math.toRadians(90))
                        .waitSeconds(0.5)
                        .strafeToConstantHeading(thirdReleasePos)
                        .waitSeconds(0.5)
                        .strafeTo(clipPos)
                        .waitSeconds(0.5)
                        .strafeToConstantHeading(fourthReleasePos)
                        .build());
        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_OFFICIAL)
                .setDarkMode(false)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}
