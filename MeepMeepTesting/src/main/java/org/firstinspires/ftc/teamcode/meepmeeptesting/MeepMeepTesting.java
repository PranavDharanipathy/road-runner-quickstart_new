package org.firstinspires.ftc.teamcode.meepmeeptesting;


import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15.5)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(44.5, -60, Math.toRadians(90)))
                .splineToLinearHeading(new Pose2d(6+22,-37.5, Math.toRadians(-60)), Math.toRadians(90))
                .waitSeconds(0.85)
                .splineToLinearHeading(new Pose2d(6+22,-34.5, Math.toRadians(-135)), Math.toRadians(-60))
                .waitSeconds(0.85)
                .splineToLinearHeading(new Pose2d(6+22,-34.5, Math.toRadians(-55)), Math.toRadians(-135))
                .splineToConstantHeading(new Vector2d(-0.75+22,-29), Math.toRadians(-135))
                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}