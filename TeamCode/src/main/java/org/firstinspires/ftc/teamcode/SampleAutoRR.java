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

        Pose2d startPose = new Pose2d(-10, -65, Math.PI/2);

        drive.setPoseEstimate(startPose);

        double moveBinSleep = 4;
        double slidesUp = 2.8;
        double outtakeServoDown = 1;
        double clawOpen = 0.6;
        double slidesDown = 2.5;
        double moveToFirstSample = 1;
        double moveToSecondSample = 1;
        double extendSlides = 0.5;
        double diffVertTime = 0.5;
        double closeAndOpenIntakeClaw = 0.6;
        double passingSample = 0.4;
        double firstSampleToBin = 1;

        TrajectorySequence mainTrajSeq = drive.trajectorySequenceBuilder(startPose)
                .addTemporalMarker(0, () -> {
                    r.horizontalSlideLeft.setPosition(r.axonServoAngle(0));
                    r.horizontalSlideRight.setPosition(r.axonServoAngle(0));
                })
                // move close to the bin then go backwards cuz if not robot will clip into the wall
                .splineToLinearHeading(new Pose2d(-48, -55, Math.PI/4), 3*Math.PI/4) // 2sqrt2 away from the bin
                // move to the bin
                .lineToLinearHeading(new Pose2d(-56.75, -62.75, Math.PI/4))
                // move slides up
                .addTemporalMarker(4, () -> {
                    r.positionTopOuttake();
                })
                .waitSeconds(slidesUp)
                .addTemporalMarker(moveBinSleep + slidesUp, () -> {
                    // outtake position drop
                    r.outtakePositionServoLeft.setPosition(0.75);
                    r.outtakePositionServoRight.setPosition(0.75);
                })
                .waitSeconds(outtakeServoDown)
                .addTemporalMarker(moveBinSleep + slidesUp + outtakeServoDown, () -> {
                    // outtake claw open
                    r.outtakeClawServo.setPosition(0.35);
                })
                .waitSeconds(clawOpen)
                // slides are moving down at the same time the bot is moving to first sample position
                .addTemporalMarker(moveBinSleep + slidesUp + outtakeServoDown + clawOpen, () -> {
                    r.outtakePositionServoLeft.setPosition(0.17);
                    r.outtakePositionServoRight.setPosition(0.17);
                    // also open outtake claw preemptively
                    r.outtakeClawServo.setPosition(0.35);
                    r.resetOuttakeSlides();
                })
                // move to first sample
                .lineToLinearHeading(new Pose2d(-46.3, -52, Math.PI/2))
                .waitSeconds(moveToFirstSample)
                // extend horizontal slides and open intake claw
                .addTemporalMarker(moveBinSleep + slidesUp + outtakeServoDown + clawOpen + moveToFirstSample, () -> { // extend differential claw
                    r.horizontalSlideRight.setPosition(r.axonServoAngle(50));
                    r.horizontalSlideLeft.setPosition(r.axonServoAngle(50));
                    // also open intake servo
                    r.intakeClawServo.setPosition(0.37);
                })
                .waitSeconds(extendSlides)
                // move diff servo down
                .addTemporalMarker(6.8 + outtakeServoDown + clawOpen + moveToFirstSample + extendSlides, () -> { // extend differential claw
                    r.passoverServoLeft.setPosition(r.axonServoAngle(228));
                    r.passoverServoRight.setPosition(r.axonServoAngle(228));
                })
                .waitSeconds(diffVertTime)
                // close intake claw ---- +0.3 might fuck up the whole thing but it is what it is
                .addTemporalMarker(6.8 + 0.3 + outtakeServoDown + clawOpen + moveToFirstSample + extendSlides + diffVertTime, () -> {
                    r.intakeClawServo.setPosition(0.15);
                })
                .waitSeconds(closeAndOpenIntakeClaw)
                // raise the diff vert and retract linear slide
                .addTemporalMarker(6.8 + outtakeServoDown + clawOpen + moveToFirstSample + extendSlides + diffVertTime + closeAndOpenIntakeClaw, () -> {
                    r.horizontalSlideRight.setPosition(r.axonServoAngle(0));
                    r.horizontalSlideLeft.setPosition(r.axonServoAngle(0));
                    r.passoverServoRight.setPosition(r.axonServoAngle(0));
                    r.passoverServoLeft.setPosition(r.axonServoAngle(0));
                })
                .waitSeconds(passingSample) // retract and extend same time dont matter
                .addTemporalMarker(6.8 + 0.3 + outtakeServoDown + clawOpen + moveToFirstSample + extendSlides + diffVertTime + closeAndOpenIntakeClaw + extendSlides, () -> {
                    // grab the sample
                    // close outtake claw
                    r.outtakeClawServo.setPosition(0.15);
                    // let go of the intake claw
                    r.intakeClawServo.setPosition(0.35);
                    // immediately start moving
                })
                // move back to bin to immediately move the linear slides up
                .lineToLinearHeading(new Pose2d(-57.75, -63.75, Math.PI/4))
                // to save time start the linear slides going up already; wait for firstSampleToBin
                .addTemporalMarker(6.8 + outtakeServoDown + clawOpen + moveToFirstSample + extendSlides + diffVertTime + closeAndOpenIntakeClaw + extendSlides + passingSample, () -> {
                    r.positionTopOuttake();
                })
                // wait for the movetobin and top outtake to finish, then drop the outtake servo
                .waitSeconds(slidesUp) // they are moving at the same time so just sleep for the longer one
                // drop outtake servo
                .addTemporalMarker(6.8 + 0.3 + outtakeServoDown + clawOpen + moveToFirstSample + extendSlides + diffVertTime + closeAndOpenIntakeClaw + extendSlides + passingSample + firstSampleToBin, () -> {
                    r.outtakePositionServoLeft.setPosition(0.72);
                    r.outtakePositionServoRight.setPosition(0.72);
                })
                .waitSeconds(outtakeServoDown)
                // open outtake claw and start moving the slides down
                .addTemporalMarker(6.8 + 0.3 +outtakeServoDown + clawOpen + moveToFirstSample + extendSlides + diffVertTime + closeAndOpenIntakeClaw + extendSlides + passingSample + firstSampleToBin + outtakeServoDown, () -> {
                    r.outtakeClawServo.setPosition(0.35);
                    r.resetOuttakeSlides();
                })
                // wait a tiny second for the outtake claw to open, the move the outtake position back down
                .waitSeconds(clawOpen)
                .addTemporalMarker(6.8 + 0.3 +outtakeServoDown + clawOpen + moveToFirstSample + extendSlides + diffVertTime + closeAndOpenIntakeClaw + extendSlides + passingSample + firstSampleToBin + outtakeServoDown + clawOpen, () -> {
                    r.outtakeClawServo.setPosition(0.35);
                    r.resetOuttakeSlides();
                })
                // move robot to second sample position
                .lineToLinearHeading(new Pose2d(-57.3, -51, Math.PI/2))
                .waitSeconds(moveToSecondSample)
                // SECOND TIME AROUND BIG MARKER
                .addTemporalMarker(6.8 + 0.3 + outtakeServoDown + clawOpen + moveToFirstSample + extendSlides + diffVertTime + closeAndOpenIntakeClaw + extendSlides + passingSample + firstSampleToBin + outtakeServoDown + clawOpen + moveToSecondSample, () -> { // extend differential claw
                    r.horizontalSlideRight.setPosition(r.axonServoAngle(50));
                    r.horizontalSlideLeft.setPosition(r.axonServoAngle(50));
                    // also open intake servo
                    r.intakeClawServo.setPosition(0.35);
                })
                .waitSeconds(extendSlides)
                // move diff servo down
                .addTemporalMarker(6.8 + 0.3 + outtakeServoDown + clawOpen + moveToFirstSample + extendSlides + diffVertTime + closeAndOpenIntakeClaw + extendSlides + passingSample + firstSampleToBin + outtakeServoDown + clawOpen + moveToSecondSample + extendSlides, () -> { // extend differential claw
                    r.passoverServoLeft.setPosition(r.axonServoAngle(228));
                    r.passoverServoRight.setPosition(r.axonServoAngle(228));
                })
                .waitSeconds(diffVertTime)
                // close intake claw
                .addTemporalMarker(6.8 + 0.3 + outtakeServoDown + clawOpen + moveToFirstSample + extendSlides + diffVertTime + closeAndOpenIntakeClaw + extendSlides + passingSample + firstSampleToBin + outtakeServoDown + clawOpen + moveToSecondSample + extendSlides + diffVertTime, () -> {
                    r.intakeClawServo.setPosition(0.15);
                })
                .waitSeconds(closeAndOpenIntakeClaw)
                // raise the diff vert and retract linear slide
                .addTemporalMarker(6.8 + 0.3 + outtakeServoDown + clawOpen + moveToFirstSample + extendSlides + diffVertTime + closeAndOpenIntakeClaw + extendSlides + passingSample + firstSampleToBin + outtakeServoDown + clawOpen + moveToSecondSample + extendSlides + diffVertTime + closeAndOpenIntakeClaw, () -> {
                    r.horizontalSlideRight.setPosition(r.axonServoAngle(0));
                    r.horizontalSlideLeft.setPosition(r.axonServoAngle(0));
                    r.passoverServoRight.setPosition(r.axonServoAngle(0));
                    r.passoverServoLeft.setPosition(r.axonServoAngle(0));
                })
                .waitSeconds(passingSample) // retract and extend same time dont matter
                .addTemporalMarker(6.8 + 0.3 + outtakeServoDown + clawOpen + moveToFirstSample + extendSlides + diffVertTime + closeAndOpenIntakeClaw + extendSlides + passingSample + firstSampleToBin + outtakeServoDown + clawOpen + moveToSecondSample + extendSlides + diffVertTime + closeAndOpenIntakeClaw + extendSlides, () -> {
                    // grab the sample
                    // close outtake claw
                    r.outtakeClawServo.setPosition(0.15);
                    // let go of the intake claw
                    r.intakeClawServo.setPosition(0.35);
                    // immediately start moving
                })
                // move back to bin to immediately move the linear slides up
                .lineToLinearHeading(new Pose2d(-56.75, -62.75, Math.PI/4))
                // to save time start the linear slides going up already; wait for firstSampleToBin
                .addTemporalMarker(6.8 + 0.3 + outtakeServoDown + clawOpen + moveToFirstSample + extendSlides + diffVertTime + closeAndOpenIntakeClaw + extendSlides + passingSample + firstSampleToBin + outtakeServoDown + clawOpen + moveToSecondSample + extendSlides + diffVertTime + closeAndOpenIntakeClaw + extendSlides + passingSample, () -> {
                    r.positionTopOuttake();
                })
                // wait for the movetobin and top outtake to finish, then drop the outtake servo
                .waitSeconds(slidesUp) // they are moving at the same time so just sleep for the longer one
                // drop outtake servo
                .addTemporalMarker(6.8 + 0.3 + outtakeServoDown + clawOpen + moveToFirstSample + extendSlides + diffVertTime + closeAndOpenIntakeClaw + extendSlides + passingSample + firstSampleToBin + outtakeServoDown + clawOpen + moveToSecondSample + extendSlides + diffVertTime + closeAndOpenIntakeClaw + extendSlides + passingSample + firstSampleToBin, () -> {
                    r.outtakePositionServoLeft.setPosition(0.72);
                    r.outtakePositionServoRight.setPosition(0.72);
                })
                .waitSeconds(outtakeServoDown)
                // open outtake claw and start moving the slides down
                .addTemporalMarker(6.8 + 0.3 + outtakeServoDown + clawOpen + moveToFirstSample + extendSlides + diffVertTime + closeAndOpenIntakeClaw + extendSlides + passingSample + firstSampleToBin + outtakeServoDown + clawOpen + moveToSecondSample + extendSlides + diffVertTime + closeAndOpenIntakeClaw + extendSlides + passingSample + firstSampleToBin + outtakeServoDown, () -> {
                    r.outtakeClawServo.setPosition(0.35);
                    r.resetOuttakeSlides();
                })
                // wait a tiny second for the outtake claw to open, the move the outtake position back down
                .waitSeconds(clawOpen)
                .addTemporalMarker(6.8 + 0.3 + outtakeServoDown + clawOpen + moveToFirstSample + extendSlides + diffVertTime + closeAndOpenIntakeClaw + extendSlides + passingSample + firstSampleToBin + outtakeServoDown + clawOpen + moveToSecondSample + extendSlides + diffVertTime + closeAndOpenIntakeClaw + extendSlides + passingSample + firstSampleToBin + outtakeServoDown + clawOpen, () -> {
                    r.outtakeClawServo.setPosition(0.35);
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
