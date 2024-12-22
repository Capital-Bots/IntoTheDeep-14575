package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedLight;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;
public class MyClass {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);
        Vector2d basketPos = new Vector2d(-56, -56);
        Vector2d releasePos = new Vector2d(-58,-58);
        Vector2d firstPiecePos = new Vector2d(-48.5,-33.25);
        Vector2d secondPiecePos = new Vector2d(-57,-33.25);
//        Vector2d thirdPiecePos = new Vector2d()
        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setColorScheme(new ColorSchemeRedLight())
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .setDimensions(17.5, 14.5)
                .setStartPose(new Pose2d(-34, -63, Math.toRadians(90)))
                .build();
        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(-34, -63, Math.toRadians(90)))
                .strafeTo(basketPos)
                .waitSeconds(1)
                .turn(Math.toRadians(-45))
                .waitSeconds(1)
                .strafeTo(releasePos)
                .waitSeconds(1)
                //release preload
                .strafeTo(basketPos)
                .waitSeconds(1)
                .strafeToSplineHeading(firstPiecePos, Math.toRadians(90)) //grab first sample
                .waitSeconds(1)
                .strafeToSplineHeading(basketPos, Math.toRadians(45))
                .waitSeconds(1)
                .strafeToSplineHeading(releasePos, Math.toRadians(45))
                .waitSeconds(1)
                .strafeToSplineHeading(basketPos, Math.toRadians(45))
                .waitSeconds(1)
                .strafeToSplineHeading(secondPiecePos, Math.toRadians(90))
                .waitSeconds(1)
                //drop sample
                .strafeToSplineHeading(basketPos, Math.toRadians(45))
                .waitSeconds(1)
                .strafeToSplineHeading(releasePos, Math.toRadians(45))
                .waitSeconds(1)
                .strafeToSplineHeading(basketPos, Math.toRadians(45))
                .waitSeconds(1)
                //drop sample
//                THIRD SAMPLE
                .splineToLinearHeading(new Pose2d(-55, -36, Math.toRadians(90)), Math.PI/2)
                .waitSeconds(1)
                .splineToLinearHeading(new Pose2d(-34, -9, Math.toRadians(90)), Math.PI/2)
                .waitSeconds(1)
                .splineToLinearHeading(new Pose2d(-25, -12, Math.toRadians(90)), Math.PI/2)
                .waitSeconds(1)
                .build());
        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_OFFICIAL)
                .setDarkMode(false)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}