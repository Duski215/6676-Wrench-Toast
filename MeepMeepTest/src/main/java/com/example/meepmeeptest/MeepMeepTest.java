package com.example.meepmeeptest;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.rowlandhall.meepmeep.MeepMeep;
import org.rowlandhall.meepmeep.roadrunner.DefaultBotBuilder;
import org.rowlandhall.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTest {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(500);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(38, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(-10, -65, Math.PI/2))
//                        .splineToLinearHeading(new Pose2d(-10, -55, Math.PI/2), 3*Math.PI/4)
//                        .build());
                        .splineToLinearHeading(new Pose2d(10, 10, Math.PI/4), 0) // 2sqrt2 away from the bin

//                        .splineToLinearHeading(new Pose2d(-50, -50, Math.PI/4), 3*Math.PI/4) // 2sqrt2 away from the bin
//                        // move the linear slides and the actual outtake to the bin
//                        .splineToLinearHeading(new Pose2d(-52, -52, Math.PI/4), 3*Math.PI/4)
//                        // drop the shit off then spline towards the three sample pieces
//                        .splineToLinearHeading(new Pose2d(-48, -45, Math.PI/2), Math.PI/2)
//                        .forward(4)
                        .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTOTHEDEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}