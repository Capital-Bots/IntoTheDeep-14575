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
        Vector2d otherBasketPos = new Vector2d(56, 56);
        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setColorScheme(new ColorSchemeRedLight())
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .setDimensions(17.5, 14.5)
                .setStartPose(new Pose2d(-34, -63, Math.toRadians(90)))
                .build();
        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(-34, -63, Math.toRadians(90)))
                .strafeTo(basketPos)
                .turn(Math.toRadians(-45))
                //drop preload
                .turn(Math.toRadians(45))
                .strafeTo(new Vector2d(-48.5, -32)) //grab first sample
                .strafeTo(basketPos)
                .turn(Math.toRadians(-45))
                //drop sample
                .turn(Math.toRadians(45))
                .strafeTo(new Vector2d(-57, -32))
                .strafeTo(basketPos)
                .turn(Math.toRadians(-45))
                //drop sample
                .turn(Math.toRadians(45))
//                THIRD SAMPLE
//                .lineToY(-26)
//                .turn(Math.toRadians(90))
//                .lineToX(-60)
//                .turn(Math.toRadians(-90))
//                .strafeTo(basketPos)
//                .turn(Math.toRadians(-45))
                //drop sample
                .turn(Math.toRadians(90))
                .strafeTo(new Vector2d(-22, 0))
                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_OFFICIAL)
                .setDarkMode(false)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}