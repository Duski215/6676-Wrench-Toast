package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(name="road runner auto")
public class RoadRunnerAuto extends LinearOpMode {
    public Robot r;

    @Override
    public void runOpMode() {
        r = new Robot(hardwareMap, telemetry);

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startPose = new Pose2d(-10, -65, Math.PI/2);

        drive.setPoseEstimate(startPose);

        TrajectorySequence mainTrajSeq = drive.trajectorySequenceBuilder(startPose)
                .addTemporalMarker(0, () -> {
                    r.horizontalSlideLeft.setPosition(r.axonServoAngle(0));
                    r.horizontalSlideRight.setPosition(r.axonServoAngle(0));
                })
                .splineToLinearHeading(new Pose2d(-50, -57, Math.PI/4), 3*Math.PI/4) // 2sqrt2 away from the bin
                // move the linear slides and the actual outtake to the bin
                .splineToLinearHeading(new Pose2d(-59.75, -65.75, Math.PI/4), 3*Math.PI/4)
                // move slides up
                .addTemporalMarker(5, () -> {
                    r.positionTopOuttake();
                    // outtake position drop
                    r.outtakePositionServoLeft.setPosition(0.75);
                    r.outtakePositionServoRight.setPosition(0.75);
                    sleep(1000); // wait 1 second for the outtake jawn to finish
                    // open claw
                    r.outtakeClawServo.setPosition(0.35);
                    sleep(500);
                    // bring the outtake position back
                    r.outtakePositionServoLeft.setPosition(0.17);
                    r.outtakePositionServoRight.setPosition(0.17);
                    sleep(1000);
                    // bring the motors back down
                    r.resetOuttakeSlides();
                })
                // this entire thing took
                .build();

//        TrajectorySequence mainTrajSeq = drive.trajectorySequenceBuilder(startPose)
//                .lineToLinearHeading(new Pose2d(20,20))
//                .build();

        waitForStart();

        if (isStopRequested()) return;

        drive.followTrajectorySequence(mainTrajSeq);
    }
}
