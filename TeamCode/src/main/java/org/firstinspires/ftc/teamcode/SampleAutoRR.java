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
        double moveToThirdSample = 2;
        double extendSlides = 0.5;
        double diffVertTime = 0.5;
        double closeAndOpenIntakeClaw = 0.6;
        double passoverTime = 0.4;
        double firstSampleToBin = 1;

        Pose2d startPose = new Pose2d(-10, -65, Math.PI/2);
        Pose2d closeToBin = new Pose2d(-48, -55, Math.PI/4);
        Pose2d atBin = new Pose2d(-55.75, -63.75, Math.PI/4);
        Pose2d firstSamplePosition = new Pose2d(-46.5, -52, Math.PI/2);
        Pose2d secondSamplePosition = new Pose2d(-55.5, -53.3, Math.PI/2);
        Pose2d thirdSamplePosition = new Pose2d(-46.5, -33, Math.PI);

        drive.setPoseEstimate(startPose);

        TrajectorySequence mainTrajSeq = drive.trajectorySequenceBuilder(startPose)
                .addTemporalMarker(0, () -> {
                    r.retractHorizontalSlides();
                    r.closeOuttake();
                    r.resetOuttakeServo();
                })
                // move close to the bin then go backwards cuz if not robot will clip into the wall
                .splineToLinearHeading(closeToBin, 3*Math.PI/4) // 2sqrt2 away from the bin
                // move to the bin
                .lineToLinearHeading(atBin)
                // move slides up
                .addTemporalMarker(1, () -> {
                    r.positionTopOuttake();
                })
//                .waitSeconds(slidesUp) // dont wait for slides up
                .addTemporalMarker(3, () -> {
                    // outtake position drop
                    r.outtakeServoTopDropoff();
                })
                .waitSeconds(outtakeServoDown-1) // wait less time for outtake servo bc happening simult
                .addTemporalMarker(3.60, () -> { // this code is looking more and more garbage the more i try random shit to make it work
                    // outtake claw open
                    r.openOuttake();
                })
                .waitSeconds(openOuttakeTime-0.7) // COMEBACK
                // slides are moving down at the same time the bot is moving to first sample position
                .addTemporalMarker(5, () -> {
                    r.resetOuttakeServo();
                    // also open outtake claw preemptively
                    r.openOuttake();
                    r.resetOuttakeSlides();
                })
                // move to first sample
                .lineToLinearHeading(firstSamplePosition)
                .waitSeconds(moveToFirstSample)
                // extend horizontal slides and open intake claw
                .addTemporalMarker(5.9, () -> { // extend differential claw
                    r.extendHorizontalSlides();
                    r.openIntake();
                })
                // now it might be too fast, gotta add a buffer
                .waitSeconds(1.1) // this is a terrible way to debug but i dont give a fuck
//                .waitSeconds(extendSlides)
                // move diff servo down
                .addTemporalMarker(6.4, () -> { // extend differential claw
                    r.dropDiffIntake();
                })
//                .waitSeconds(diffVertTime)
                // close intake claw
                .addTemporalMarker(6.9, () -> {
                    r.closeIntake();
                })
                .waitSeconds(closeAndOpenIntakeClaw)
                // AS SOON AS WE GRABBED THE SAMPLE, JUST MOVE BACK TO BIN
                .lineToLinearHeading(atBin)
                // EVERYTHING ELSE IS HAPPENING SIMULTANEOUSLY, IN TRAJECTORY ONLY WAIT FOR TOBIN
                // however, in the temporal markers, make sure u do "wait" for each command to finish
                // raise diff servo
                .addTemporalMarker(7.4, () -> {
                    r.raiseDiffIntake();
                })
                // raise the diff vert and retract linear slide
                .addTemporalMarker(7.9, () -> {
                    r.retractHorizontalSlides();
                })
                // just wait for retract slides IN TEMPORAL MARKER
                // passover sample
                .addTemporalMarker(8.4, () -> {
                    r.closeOuttake();
                    r.openIntake();
                    // immediately start moving, no waiting
                })
                // to save time start the linear slides going up already; wait for firstSampleToBin
                .addTemporalMarker(8.5, () -> {
                    r.positionTopOuttake();
                })
                // wait for the movetobin and top outtake to finish, then drop the outtake servo
                // dont wait for slides up because its happening simultaneously
                // drop outtake servo
                .addTemporalMarker(9.5, () -> {
                    r.outtakeServoTopDropoff();
                })
                .waitSeconds(outtakeServoDown)
                // open outtake claw
                .addTemporalMarker(10.2, () -> {
                    r.openOuttake();
                })
                // wait a tiny second for the outtake claw to open, the move the outtake position back down
                .waitSeconds(openOuttakeTime)
                .addTemporalMarker(11.4, () -> {
                    r.resetOuttakeServo();
                })
                // wait just a tiny second for the outtake servo to get out the way enough for the slides to come down
                .waitSeconds(0.1)
                .addTemporalMarker(11.7, () -> {
                    r.resetOuttakeSlides();
                })
                // move robot to second sample position
                .lineToLinearHeading(secondSamplePosition)
                // BUMP OFF 0.4 FROM HERE COMEBACK
                // SECOND TIME AROUND BIG MARKER
                .addTemporalMarker(12.8, () -> { // extend horizontal slides
                    r.extendHorizontalSlides();
                    // also open intake servo
                    r.openIntake();
                })
                .waitSeconds(extendSlides)
                // move diff servo down
                .addTemporalMarker(13.2, () -> { // extend differential claw
                    r.dropDiffIntake();
                })
                .waitSeconds(diffVertTime)
                // close intake claw
                .addTemporalMarker(13.6, () -> {
                    r.closeIntake();
                })
                .waitSeconds(closeAndOpenIntakeClaw)
                // raise the diff vert
                .addTemporalMarker(14.0, () -> {
                    r.raiseDiffIntake();
                })
                .waitSeconds(diffVertTime)
                // retract linear slide
                .addTemporalMarker(14.2, () -> {
                    r.retractHorizontalSlides();
                })
                .waitSeconds(extendSlides+0.05) // retract and extend same time dont matter
                .addTemporalMarker(14.8, () -> {
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
                .addTemporalMarker(15.0, () -> {
                    r.positionTopOuttake();
                })
                // wait for the movetobin and top outtake to finish, then drop the outtake servo
                .waitSeconds(slidesUp) // they are moving at the same time so just sleep for the longer one
                // drop outtake servo
                .addTemporalMarker(17.0, () -> {
                    r.outtakeServoTopDropoff();
                })
                .waitSeconds(outtakeServoDown)
                // open outtake claw and start moving the slides down
                .addTemporalMarker(17.8, () -> {
                    r.openOuttake();
                })
                // wait a tiny second for the outtake claw to open, the move the outtake position back down. also open outtake claw preemptively
                .waitSeconds(openOuttakeTime)
                .addTemporalMarker(18.8, () -> {
                    r.resetOuttakeServo();
                    // open it preemptively for the third sample to come in during passover
                })
                // wait a tiny second then reset outtake to avoid clipping onto the top bin
                .waitSeconds(0.3)
                .addTemporalMarker(18.9, () -> {
                    r.resetOuttakeSlides();
                })
                // SIMULTANEOUSLY drop slides and move to third sample. extend slides -- thats all for current test
                .lineToLinearHeading(thirdSamplePosition)
                .addTemporalMarker(21.5, () -> {
                    r.extendHorizontalSlides();
                })
                // extend + 0.2, then pivot
                .addTemporalMarker(22.2, () -> {
                    r.pivotPassover();
                })
                // wait diffVertTime, then grab command
                .addTemporalMarker(22.7, () -> {
                    r.closeIntake();
                })
                // wait grab time, run retract and raise diff intake at the same time simultaneously
                .addTemporalMarker(23.3, () -> {
                    // COMEBACK if this doesnt work in second sample
                    r.raiseDiffIntake();
                    r.retractHorizontalSlides();
                })
                // move back to at bin
                // TOTAL TRAJECTORY WAIT: MOVETOTHIRDSAMPLE, EXTEND, PIVOT+0.2, GRAB
                .waitSeconds(moveToThirdSample + extendSlides + diffVertTime + 0.2 + closeAndOpenIntakeClaw - 2)
                .lineToLinearHeading(atBin)
                .addTemporalMarker(23.8, () -> {
                    // passover jawn
                    r.openIntake();
                    r.closeOuttake();
                })
                .addTemporalMarker(24.2, () -> {
                    // raise slides
                    r.positionTopOuttake();
                })
                .addTemporalMarker(26.0, () -> {
                    // raise outtake servo
                    r.outtakeServoTopDropoff();
                })
                .addTemporalMarker(27.0, () -> {
                    // open outtake claw
                    r.openOuttake();
                })
                .addTemporalMarker(28, () -> {
                    // reset outtake and drop the slides down
                    r.resetOuttakeServo();
                    r.resetOuttakeSlides();
                })
                .waitSeconds(20) // just so it doesnt stop before i want it to
                .build();

//        TrajectorySequence mainTrajSeq = drive.trajectorySequenceBuilder(startPose)
//                .lineToLinearHeading(new Pose2d(20,20))
//                .build();

        waitForStart();

        if (isStopRequested()) return;

        drive.followTrajectorySequence(mainTrajSeq);
    }
}
