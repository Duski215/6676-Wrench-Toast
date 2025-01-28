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

        Pose2d startPose = new Pose2d(7.75, -62, Math.PI/2);

        drive.setPoseEstimate(startPose);

        TrajectorySequence mainTrajSeq = drive.trajectorySequenceBuilder(startPose)
                .addTemporalMarker(0, () -> {
                    r.horizontalSlideLeft.setPosition(r.axonServoAngle(0));
                    r.horizontalSlideRight.setPosition(r.axonServoAngle(0));
                })
                .lineToLinearHeading(new Pose2d(4,-35, (3*Math.PI)/2))
                .splineToLinearHeading(new Pose2d(35, -25, Math.PI/2), Math.PI/2)

                .splineToConstantHeading(new Vector2d(47,-10), Math.toRadians(0))
                .lineToLinearHeading(new Pose2d(47,-55,Math.PI/2))

                .lineToLinearHeading(new Pose2d(47,-10,Math.PI/2))
                //issue
                .splineToConstantHeading(new Vector2d(58,-14), Math.toRadians(0))

                .lineToLinearHeading(new Pose2d(58,-56,Math.PI/2))

                .lineToLinearHeading(new Pose2d(56,-10,Math.PI/2))
                //issue
                .splineToConstantHeading(new Vector2d(66,-14), Math.toRadians(0))

                .lineToLinearHeading(new Pose2d(66,-59,Math.PI/2))

                .lineToLinearHeading(new Pose2d(35,-55,Math.PI/2))
                .lineToLinearHeading(new Pose2d(4,-35, (3*Math.PI)/2))
                .lineToLinearHeading(new Pose2d(35,-55,Math.PI/2))

                .lineToLinearHeading(new Pose2d(4,-35, (3*Math.PI)/2))
                .lineToLinearHeading(new Pose2d(35,-55,Math.PI/2))

                .lineToLinearHeading(new Pose2d(4,-35, (3*Math.PI)/2))
                .lineToLinearHeading(new Pose2d(35,-55,Math.PI/2))

                .build();

//        TrajectorySequence mainTrajSeq = drive.trajectorySequenceBuilder(startPose)
//                .lineToLinearHeading(new Pose2d(20,20))
//                .build();

        waitForStart();

        if (isStopRequested()) return;

        drive.followTrajectorySequence(mainTrajSeq);
    }
}
