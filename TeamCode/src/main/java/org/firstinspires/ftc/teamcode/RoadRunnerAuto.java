package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Autonomous(name="road runner auto")
public class RoadRunnerAuto extends LinearOpMode {
    @Override
    public void runOpMode() {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Trajectory rightTrajectory = drive.trajectoryBuilder(new Pose2d())
            .strafeRight(10).build();

        Trajectory forwardTrajectory = drive.trajectoryBuilder(new Pose2d())
                .forward(5).build();

        waitForStart();

        if (isStopRequested()) return;

        drive.followTrajectory(rightTrajectory);
        drive.followTrajectory(forwardTrajectory);
    }
}
