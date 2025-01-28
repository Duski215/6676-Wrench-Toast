package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(name="Sample 1+3")
public class SampleAutoRR extends LinearOpMode {
    public Robot r;

    double timeStamp1 = 0;

    @Override
    public void runOpMode() {
        r = new Robot(hardwareMap, telemetry);

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        double moveBinSleep = 4;
        double slidesUp = 2.8;
        double outtakeServoDown = 1;
        double openOuttakeTime = 0.6;
        double slidesDown = 2.5;
        double moveToFirstSample = 1;
        double moveToSecondSample = 1;
        double extendSlides = 0.5;
        double diffVertTime = 0.5;
        double closeAndOpenIntakeClaw = 0.6;
        double passoverTime = 0.4;
        double firstSampleToBin = 1;

        Pose2d startPose = new Pose2d(-10, -65, Math.PI/2);
        Pose2d closeToBin = new Pose2d(-48, -55, Math.PI/4);
        Pose2d atBin = new Pose2d(-56.75, -62.75, Math.PI/4);
        Pose2d firstSamplePosition = new Pose2d(-46.3, -52, Math.PI/2);
        Pose2d secondSamplePosition = new Pose2d(-57.3, -51, Math.PI/2);

        drive.setPoseEstimate(startPose);

        TrajectorySequence mainTrajSeq = drive.trajectorySequenceBuilder(startPose)
                .addTemporalMarker(0, () -> {
                    r.retractHorizontalSlides();
                })
                // move close to the bin then go backwards cuz if not robot will clip into the wall
                .splineToLinearHeading(closeToBin, 3*Math.PI/4) // 2sqrt2 away from the bin
                // move to the bin
                .lineToLinearHeading(atBin)
                // move slides up
                .addTemporalMarker(4, () -> {
                    r.positionTopOuttake();
                })
                .waitSeconds(slidesUp)
                .addTemporalMarker(6.8, () -> {
                    // outtake position drop
                    r.outtakeServoTopDropoff();
                })
                .waitSeconds(outtakeServoDown)
                .addTemporalMarker(7.8, () -> {
                    // outtake claw open
                    r.openOuttake();
                })
                .waitSeconds(openOuttakeTime)
                // slides are moving down at the same time the bot is moving to first sample position
                .addTemporalMarker(8.4, () -> {
                    r.resetOuttakeServo();
                    // also open outtake claw preemptively
                    r.openOuttake();
                    r.resetOuttakeSlides();
                })
                // move to first sample
                .lineToLinearHeading(firstSamplePosition)
                .waitSeconds(moveToFirstSample)
                // extend horizontal slides and open intake claw
                .addTemporalMarker(9.4, () -> { // extend differential claw
                    r.extendHorizontalSlides();
                    r.openIntake();
                })
                .waitSeconds(extendSlides)
                // move diff servo down
                .addTemporalMarker(9.9, () -> { // extend differential claw
                    r.dropDiffIntake();
                })
                .waitSeconds(diffVertTime)
                // close intake claw
                .addTemporalMarker(10.4, () -> {
                    r.closeIntake();
                })
                .waitSeconds(closeAndOpenIntakeClaw)
                // raise diff servo
                .addTemporalMarker(10.9, () -> {
                    r.raiseDiffIntake();
                })
                .waitSeconds(diffVertTime)
                // raise the diff vert and retract linear slide
                .addTemporalMarker(11.5, () -> {
                    r.retractHorizontalSlides();
                })
                .waitSeconds(passoverTime) // retract and extend same time dont matter
                .addTemporalMarker(12.3, () -> {
                    r.closeOuttake();
                    r.openIntake();
                    // immediately start moving, no waiting
                })
                // move back to bin to immediately move the linear slides up
                .lineToLinearHeading(atBin)
                // to save time start the linear slides going up already; wait for firstSampleToBin
                .addTemporalMarker(12.4, () -> {
                    r.positionTopOuttake();
                })
                // wait for the movetobin and top outtake to finish, then drop the outtake servo
                .waitSeconds(slidesUp) // they are moving at the same time so just sleep for the longer one
                // drop outtake servo
                .addTemporalMarker(13.7, () -> {
                    r.outtakeServoTopDropoff();
                })
                .waitSeconds(outtakeServoDown)
                // open outtake claw
                .addTemporalMarker(14.7, () -> {
                    r.openOuttake();
                })
                // wait a tiny second for the outtake claw to open, the move the outtake position back down
                .waitSeconds(openOuttakeTime)
                .addTemporalMarker(15.3, () -> {
                    r.resetOuttakeServo();
                })
                // wait just a tiny second for the outtake servo to get out the way enough for the slides to come down
                .waitSeconds(0.1)
                .addTemporalMarker(15.6, () -> {
                    r.resetOuttakeSlides();
                })
                // move robot to second sample position
                .lineToLinearHeading(secondSamplePosition)
                .waitSeconds(moveToSecondSample)

                // SECOND TIME AROUND BIG MARKER
                .addTemporalMarker(16.3, () -> { // extend horizontal slides
                    r.extendHorizontalSlides();
                    // also open intake servo
                    r.openIntake();
                })
                .waitSeconds(extendSlides)
                // move diff servo down
                .addTemporalMarker(16.8, () -> { // extend differential claw
                    r.dropDiffIntake();
                })
                .waitSeconds(diffVertTime)
                // close intake claw
                .addTemporalMarker(17.3, () -> {
                    r.closeIntake();
                })
                .waitSeconds(closeAndOpenIntakeClaw)
                // raise the diff vert
                .addTemporalMarker(17.9, () -> {
                    r.raiseDiffIntake();
                })
                .waitSeconds(diffVertTime)
                // retract linear slide
                .addTemporalMarker(18.4, () -> {
                    r.retractHorizontalSlides();
                })
                .waitSeconds(extendSlides) // retract and extend same time dont matter
                .addTemporalMarker(18.9, () -> {
                    // passover the sample
                    // close outtake claw
                    r.closeOuttake();
                    // let go of the intake claw
                    r.openIntake();
                    // immediately start moving. forget about passover time, that will be encompassed by the movetobin time
                })
                // move back to bin to immediately move the linear slides up
                .lineToLinearHeading(atBin)
                // to save time start the linear slides going up already; wait for firstSampleToBin
                // "waits" in the temporal marker for MOVETOBIN time
                .addTemporalMarker(19.9, () -> {
                    r.positionTopOuttake();
                })
                // wait for the movetobin and top outtake to finish, then drop the outtake servo
                .waitSeconds(slidesUp) // they are moving at the same time so just sleep for the longer one
                // drop outtake servo
                .addTemporalMarker(20.8, () -> {
                    r.outtakeServoTopDropoff();
                })
                .waitSeconds(outtakeServoDown)
                // open outtake claw and start moving the slides down
                .addTemporalMarker(21.8, () -> {
                    r.openOuttake();
                })
                // wait a tiny second for the outtake claw to open, the move the outtake position back down. also open outtake claw preemptively
                .waitSeconds(openOuttakeTime)
                .addTemporalMarker(22.4, () -> {
                    r.resetOuttakeServo();
                    // open it preemptively for the third sample to come in during passover
                    r.openOuttake();

                })
                // wait a tiny second then reset outtake to avoid clipping onto the top bin
                .waitSeconds(0.3)
                .addTemporalMarker(22.7, () -> {
                    r.resetOuttakeSlides();
                })
                .build();

//        TrajectorySequence mainTrajSeq = drive.trajectorySequenceBuilder(startPose)
//                .lineToLinearHeading(new Pose2d(20,20))
//                .build();

        waitForStart();

        if (isStopRequested()) return;

        drive.followTrajectorySequence(mainTrajSeq);
    }
}
