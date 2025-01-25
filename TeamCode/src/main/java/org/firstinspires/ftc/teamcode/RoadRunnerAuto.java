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
    private DcMotor motorLeftVert;
    private DcMotor motorRightVert;
    double topBinHeightInches = -28;
    double midBinHeightInches = -16;
    private Servo outtakePositionServoLeft;
    private Servo outtakePositionServoRight;
    private Servo horizontalSlideLeft;
    private Servo horizontalSlideRight;
    private Servo outtakeClawServo;

    double ticksPerRotation = 537.6;
    double spoolCircumference = 4.40945;

    double newTarget;
    double servoTarget;

    double axonServoMaxAngle = 355;

    @Override
    public void runOpMode() {
        motorLeftVert = hardwareMap.get(DcMotor.class, "exMotor0");
        motorRightVert = hardwareMap.get(DcMotor.class, "exMotor1");
        outtakeClawServo = hardwareMap.get(Servo.class, "servo1");

        motorLeftVert.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorRightVert.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motorLeftVert.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRightVert.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorLeftVert.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRightVert.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        horizontalSlideLeft = hardwareMap.get(Servo.class, "servo2");
        horizontalSlideRight = hardwareMap.get(Servo.class, "servo3");

        outtakePositionServoLeft = hardwareMap.get(Servo.class, "servo4");
        outtakePositionServoRight = hardwareMap.get(Servo.class, "servo5");

        horizontalSlideLeft.setDirection(Servo.Direction.REVERSE); // this ain't a typo it supposed to be right idk why
        outtakePositionServoLeft.setDirection(Servo.Direction.REVERSE);
        motorRightVert.setDirection(DcMotorSimple.Direction.REVERSE);

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startPose = new Pose2d(-10, -65, Math.PI/2);

        drive.setPoseEstimate(startPose);

        TrajectorySequence mainTrajSeq = drive.trajectorySequenceBuilder(startPose)
                .addTemporalMarker(0, () -> {
                    horizontalSlideLeft.setPosition(axonServoAngle(0));
                    horizontalSlideRight.setPosition(axonServoAngle(0));
                })
                .splineToLinearHeading(new Pose2d(-50, -57, Math.PI/4), 3*Math.PI/4) // 2sqrt2 away from the bin
                // move the linear slides and the actual outtake to the bin
                .splineToLinearHeading(new Pose2d(-59.75, -65.75, Math.PI/4), 3*Math.PI/4)
                // move slides up
                .addTemporalMarker(5, () -> {
                    this.positionTopOuttake();
                    // outtake position drop
                    outtakePositionServoLeft.setPosition(0.75);
                    outtakePositionServoRight.setPosition(0.75);
                    sleep(1000); // wait 1 second for the outtake jawn to finish
                    // open claw
                    outtakeClawServo.setPosition(0.35);
                    sleep(500);
                    // bring the outtake position back
                    outtakePositionServoLeft.setPosition(0.17);
                    outtakePositionServoRight.setPosition(0.17);
                    sleep(1000);
                    // bring the motors back down
                    this.resetOuttakeSlides();
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

    private double ticksToInchesSpool(double inches) {
        return inches * ticksPerRotation / spoolCircumference;
    }

    private double axonServoAngle(double degree) {
        servoTarget = degree / axonServoMaxAngle;

        return servoTarget;
    }

    public void positionTopOuttake() {
        newTarget = ticksToInchesSpool(topBinHeightInches);

        motorLeftVert.setTargetPosition((int) newTarget);
        motorRightVert.setTargetPosition((int) newTarget);

        motorLeftVert.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorRightVert.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        motorLeftVert.setPower(0.8);
        motorRightVert.setPower(0.8);

        // ONLY FOR AUTONOMOUS
        while (motorLeftVert.isBusy() && motorRightVert.isBusy()) {}

        motorLeftVert.setPower(0);
        motorRightVert.setPower(0);
    }

    public void positionMidOuttake() {
        newTarget = ticksToInchesSpool(midBinHeightInches);

        motorLeftVert.setTargetPosition((int) newTarget);
        motorRightVert.setTargetPosition((int) newTarget);

        motorLeftVert.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorRightVert.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        motorLeftVert.setPower(0.8);
        motorRightVert.setPower(0.8);

        // ONLY FOR AUTONOMOUS
        while (motorLeftVert.isBusy() && motorRightVert.isBusy()) {}

        motorLeftVert.setPower(0);
        motorRightVert.setPower(0);
    }

    //reset position
    public void resetOuttakeSlides() {
        motorLeftVert.setTargetPosition(0);
        motorRightVert.setTargetPosition(0);

        motorLeftVert.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorRightVert.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        motorLeftVert.setPower(0.6);
        motorRightVert.setPower(0.6);

        // ONLY FOR AUTONOMOUS
        while (motorLeftVert.isBusy() && motorRightVert.isBusy()) {}

        motorLeftVert.setPower(0);
        motorRightVert.setPower(0);
    }
}
