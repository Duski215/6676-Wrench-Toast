package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous (name = "Specimen 1+3")
public class SpecimenAutoRR extends LinearOpMode{
    public Robot r;

    public void runOpMode() {
        r = new Robot(hardwareMap, telemetry);

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startPose = new Pose2d(7.75, -62, (3*Math.PI)/2);

        drive.setPoseEstimate(startPose);

        TrajectorySequence mainTrajSeq = drive.trajectorySequenceBuilder(startPose)
                .addTemporalMarker(0, () -> {
                    r.horizontalSlideLeft.setPosition(r.axonServoAngle(0));
                    r.horizontalSlideRight.setPosition(r.axonServoAngle(0));
                    r.closeOuttake();
                })


                .lineToLinearHeading(new Pose2d(4,-35, (3*Math.PI)/2))

                .addTemporalMarker(0.5, () -> {
                    r.specimenOffWall();
                    r.positionMidOuttake();
                })

                .addTemporalMarker(2.75, () -> {
                    r.scoreSpecimen();
                })


                .addTemporalMarker(3.6, () -> {
                    r.openOuttake();
                })

                .waitSeconds(3.5)
                .splineToLinearHeading(new Pose2d(35, -25, Math.PI/2), Math.PI/2)
                // first sample
                .splineToConstantHeading(new Vector2d(47,-13), Math.toRadians(0))

                // brings it back
                .lineToLinearHeading(new Pose2d(47,-55,Math.PI/2))

                // round 2
                .lineToLinearHeading(new Pose2d(47,-25,Math.PI/2))
                .splineToConstantHeading(new Vector2d(56,-12), Math.toRadians(0))

                //brings back
                .lineToLinearHeading(new Pose2d(58,-55,Math.PI/2))
                .lineToLinearHeading(new Pose2d(33,-57, Math.PI/2))

                 .addTemporalMarker(13.5, () ->{
                 r.closeOuttake();
                 })

                .waitSeconds(1)
                .lineToLinearHeading(new Pose2d(4,-35, (3*Math.PI)/2))
                 .addTemporalMarker(15, () ->{
                r.positionMidOuttake();
                })

                .addTemporalMarker(17.7, () -> {
                    r.scoreSpecimen();
                })

                .addTemporalMarker(18, () -> {
                 r.openOuttake();
                })

                .addTemporalMarker(18.5, () -> {
                 r.resetOuttakeSlides();
                })

                .waitSeconds(1)

                 .addTemporalMarker(18.6, () -> {
                     r.openOuttake();
                 })

                .lineToLinearHeading(new Pose2d(33,-55, Math.PI/2))
                .lineToLinearHeading(new Pose2d(33,-57, Math.PI/2))

                /*
                .addTemporalMarker(21.6, () -> {
                 r.closeOuttake();
                })
                .addTemporalMarker(15, () ->{
                r.midBinOuttake();
                })

                .addTemporalMarker(17.7, () -> {
                    r.scoreSpecimen();
                })

                .addTemporalMarker(18, () -> {
                 r.openOuttake();
                })

                .addTemporalMarker(18.5, () -> {
                 r.resetOuttakeSlides();
                })
                 */

                .waitSeconds(3)

                .build();

//        TrajectorySequence mainTrajSeq = drive.trajectorySequenceBuilder(startPose)
//                .lineToLinearHeading(new Pose2d(20,20))
//                .build();

        waitForStart();

        if (isStopRequested()) return;

        drive.followTrajectorySequence(mainTrajSeq);
    }
}
