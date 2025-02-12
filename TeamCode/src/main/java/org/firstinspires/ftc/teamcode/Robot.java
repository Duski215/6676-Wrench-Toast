package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

//Camera stuff
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

public class Robot {
    // NOTE: if XYZ values are (0, 0, 0), then that means all the camera is at the dead center of the robot.
    // find all distances from the *center of the robot (the height of the "center" of the robot is
    // 0 --- just measure the height of the camera off the ground. define the "center" of the robot
    // xy coordinates, then offset it to find the camera position

    // measure this before you start with the camera
    public Position cameraPosition = new Position(DistanceUnit.INCH, -6.5, 3.125, 9.25, 0); //ADJUST THESE

    // if all values are zero (no rotation), that implies the camera is pointing straight up.
    public YawPitchRollAngles cameraOrientation = new YawPitchRollAngles(AngleUnit.DEGREES, 90, -90, 0, 0);// AND MAYBE THESE

    public AprilTagProcessor aprilTag;
    public VisionPortal visionPortal;


    // Declare OpMode members for each of the 4 motors.
    /*
    motor 0  = Front_left
    Motor1  = Back_Left
    Motor2 = Front_Right
    Motor3 = Back_Right
     */
    //these are variables we declare what electronics we're using
    public ElapsedTime runtime = new ElapsedTime();
    public DcMotor frontLeftDrive = null;
    public DcMotor backLeftDrive = null;
    public DcMotor frontRightDrive = null;
    public DcMotor backRightDrive = null;
    public IMU imu = null;
    public Servo intakeClawServo;
    public Servo outtakeClawServo;
    public Servo horizontalSlideLeft;
    public Servo horizontalSlideRight;
    public Servo passoverServoLeft;
    public Servo passoverServoRight;
    public Servo hardStopServo;
    public DigitalChannel buttonSensor;


    public DcMotor motorLeftVert;
    public DcMotor motorRightVert;
    public Servo outtakePositionServoLeft;
    public Servo outtakePositionServoRight;
    public DcMotor motorHang;
    //these variables are mostly values we give values to or will give values to.
    double ticksPerRotation = 537.6;
    double wheelCircumference = 11.8737374;
    double spoolCircumference = 4.40945;
    double goBuildaMaxServoAngle = 300;
    double axonServoMaxAngle = 355;
    double servoTarget;
    double newTarget;
    //These are negatives due to how the motors are facing
    double midBinHeightInches = -16;
    double topBinHeightInches = -28;
    double midHangInches = -20;
    double topHangInches = -36;
    double topChamber = -14;
    double scoreSpecimen = -10;
    //toggleSwitch between Field Centric Drive to Robot Centric Drive
    boolean toggleSwitch = false;
    //this starts off as false, which will leave us in basket mode, we can swap into specimen
    boolean modeChange = false;

    public final double bufferTime = 400; // milliseconds
    public double prevBuffer = 0;
    public boolean buffer = false;

    private Telemetry telemetry;
    private HardwareMap hardwareMap;


    public Robot(HardwareMap hardwareMap, Telemetry telemetry) {
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;

        //if the device name you'll get an error from the driver hub tell you what's wrong
        frontLeftDrive = hardwareMap.get(DcMotor.class, "motor0");
        backLeftDrive = hardwareMap.get(DcMotor.class, "motor2");
        frontRightDrive = hardwareMap.get(DcMotor.class, "motor1");
        backRightDrive = hardwareMap.get(DcMotor.class, "motor3");

        // todo:change these as wiring has changed
        intakeClawServo = hardwareMap.get(Servo.class, "exServo0");
        outtakeClawServo = hardwareMap.get(Servo.class, "servo1");

        horizontalSlideLeft = hardwareMap.get(Servo.class, "servo2");
        horizontalSlideRight = hardwareMap.get(Servo.class, "servo3");

        passoverServoLeft = hardwareMap.get(Servo.class, "exPassoverServo4");
        passoverServoRight = hardwareMap.get(Servo.class, "exPassoverServo5");

        outtakePositionServoLeft = hardwareMap.get(Servo.class, "servo4");
        outtakePositionServoRight = hardwareMap.get(Servo.class, "servo5");

        hardStopServo = hardwareMap.get(Servo.class, "exServo2");

        motorLeftVert = hardwareMap.get(DcMotor.class, "exMotor0");
        motorRightVert = hardwareMap.get(DcMotor.class, "exMotor1");
        motorHang = hardwareMap.get(DcMotor.class, "exMotor2");

        motorLeftVert.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorRightVert.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        buttonSensor = hardwareMap.get(DigitalChannel.class, "buttonSensor");

        //because we often have motors/servos in pairs facing opposite ways we have to reverse directions
        // so they turn in the same direction
        horizontalSlideLeft.setDirection(Servo.Direction.REVERSE); // this ain't a typo it supposed to be right idk why
        passoverServoRight.setDirection(Servo.Direction.REVERSE);
        motorRightVert.setDirection(DcMotorSimple.Direction.REVERSE);
        outtakePositionServoLeft.setDirection(Servo.Direction.REVERSE);

        hardStopServo.setDirection(Servo.Direction.REVERSE); // this ain't a typo it supposed to be right idk why

        // reset encoders
        motorLeftVert.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRightVert.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        motorLeftVert.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        motorRightVert.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorHang.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // set correct motor direction
        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE); // reverse og
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE); // reverse og
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD); // forward
        backRightDrive.setDirection(DcMotor.Direction.FORWARD); // forward

        //this is a thing in the Control Hub which determines the direction of the robot
        imu = hardwareMap.get(IMU.class, "imu");
        //these determines the direction of the robot depending on how the control hub is oriented
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
        ));

        imu.initialize(parameters);
        initAprilTag();

        // Wait for the DS start button to be touched.
        telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch START to start OpMode");
        telemetry.update();

        // Wait for the game to start (driver presses START)
        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    public void stopAndResetSlides() {
        motorLeftVert.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRightVert.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    /*everything below is considered an object. It is basically something you can give qualities to
    Like a variables but can be multi-detailed. These objects can return values.
    For example in the conversions.
    We can also use objects in conditionals to do things through the traits we give it.
    For example in positionTopOuttake we give the motors traits to be at a certain position
    with a certain amount of power. And it'll only do this when we call this object in a conditional.
     */

    /*conversions
    these will convert units we understand into units the program understands.
    For example, the program understands the amount to spin a motor in ticks.
    Motors will have a certain amount of ticks per revolution
    So we can use a bit of math to find the amount of ticks per rotation of an object attached to
    a motor.
    For example, in positionTopOuttake we can use the ticksTnInchesSpool rotate the spool,
    which will then in turn move
    the slides up a certain amount of inches by
    topBinHeightInches variable which is equal to a certain amount of inches
     */
    public double ticksToInchesSpool(double inches) {
        return inches * ticksPerRotation / spoolCircumference;
    }

    public double ticksToInchesWheel(double inches) {
        return inches * ticksPerRotation / wheelCircumference;
    }

    //todo: integrate this into servo positions
    public double oldServoAngle(double degree) {
        servoTarget = degree / goBuildaMaxServoAngle;
        return servoTarget;
    }

    public double axonServoAngle(double degree) {
        servoTarget = degree / axonServoMaxAngle;

        return servoTarget;
    }

    /*
    These are objects which we can call in conditionals to do certain things
     */
    public void positionTopOuttake() {
        newTarget = ticksToInchesSpool(topBinHeightInches);

        motorLeftVert.setTargetPosition((int) newTarget);
        motorRightVert.setTargetPosition((int) newTarget);

        motorLeftVert.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorRightVert.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        motorLeftVert.setPower(0.9);
        motorRightVert.setPower(0.9);
    }

    public void positionMidOuttake() {
        newTarget = ticksToInchesSpool(midBinHeightInches);

        motorLeftVert.setTargetPosition((int) newTarget);
        motorRightVert.setTargetPosition((int) newTarget);

        motorLeftVert.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorRightVert.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        motorLeftVert.setPower(0.9);
        motorRightVert.setPower(0.9);
    }

    //reset position
    public void resetOuttakeSlides() {
        motorLeftVert.setTargetPosition(0);
        motorRightVert.setTargetPosition(0);

        motorLeftVert.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorRightVert.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        motorLeftVert.setPower(0.9);
        motorRightVert.setPower(0.9);
    }

    public void positionMidHang() {
//        motorHang.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        newTarget = ticksToInchesSpool(midHangInches);
        motorHang.setTargetPosition((int) newTarget);
        motorHang.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorHang.setPower(0.4);
    }

    public void positionTopHang() {
//        motorHang.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        newTarget = ticksToInchesSpool(topHangInches);
        motorHang.setTargetPosition((int) newTarget);
        motorHang.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorHang.setPower(0.4);
    }

    public void topChamber() {
        newTarget = ticksToInchesSpool(topChamber);
        motorLeftVert.setTargetPosition((int) newTarget);
        motorRightVert.setTargetPosition((int) newTarget);
        motorLeftVert.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorRightVert.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorRightVert.setPower(0.5);
        motorLeftVert.setPower(0.5);
    }

    public void scoreSpecimen() {
        newTarget = ticksToInchesSpool(scoreSpecimen);
        motorLeftVert.setTargetPosition((int) newTarget);
        motorRightVert.setTargetPosition((int) newTarget);
        motorLeftVert.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorRightVert.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorRightVert.setPower(0.5);
        motorLeftVert.setPower(0.5);
    }

    public void resetHang() {
        motorHang.setTargetPosition(0);
        motorHang.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorHang.setPower(0.6);
    }

    public void openIntake() {
        intakeClawServo.setPosition(0.40);
    }

    public void openIntakeAuto() {
        intakeClawServo.setPosition(0.43);
    }

    public void closeIntake() {
        intakeClawServo.setPosition(0.15);
    }

    public void openOuttake() {
        outtakeClawServo.setPosition(0.27);
    }

    public void closeOuttake() {
        outtakeClawServo.setPosition(0.10);
    }

    public void extendHorizontalSlides() {
        horizontalSlideRight.setPosition(axonServoAngle(50));
        horizontalSlideLeft.setPosition(axonServoAngle(50));
    }

    public void retractHorizontalSlides() {
        horizontalSlideLeft.setPosition(axonServoAngle(0));
        horizontalSlideRight.setPosition(axonServoAngle(0));
    }

    public void dropDiffIntakeAuto() {
        passoverServoLeft.setPosition(axonServoAngle(250));
        passoverServoRight.setPosition(axonServoAngle(250));
    }


    //was 225
    public void dropDiffIntake() {
        passoverServoLeft.setPosition(axonServoAngle(245));
        passoverServoRight.setPosition(axonServoAngle(245));
    }

    public void raiseDiffIntake() {
        passoverServoRight.setPosition(axonServoAngle(0));
        passoverServoLeft.setPosition(axonServoAngle(0));
    }

    public void pivotPassover() {
        //added ten degrees
        passoverServoRight.setPosition(axonServoAngle(315));
        passoverServoLeft.setPosition(axonServoAngle(135));
    }

    public void unpivotPassover() {
        passoverServoRight.setPosition(axonServoAngle(225));
        passoverServoLeft.setPosition(axonServoAngle(225));
    }

    public void specimenOffWall() {
        outtakePositionServoRight.setPosition(1);
        outtakePositionServoLeft.setPosition(1);
    }

    public void outtakeServoTopDropoff() {
        outtakePositionServoLeft.setPosition(0.75);
        outtakePositionServoRight.setPosition(0.75);
    }

    public void outtakeServoMidDropoff() {
        outtakePositionServoLeft.setPosition(0.97);
        outtakePositionServoRight.setPosition(0.97);
    }

    public void resetOuttakeServo() {
        outtakePositionServoLeft.setPosition(.185);
        outtakePositionServoRight.setPosition(.185);
    }

    public void hardStopDisActivate(){
        hardStopServo.setPosition(.2);
    }
    public void hardStopActive(){
        hardStopServo.setPosition(.7);
    }

    public void initAprilTag() {
        // Create the AprilTag processor.
        aprilTag = new AprilTagProcessor.Builder()

                // The following default settings are available to un-comment and edit as needed.
                //.setDrawAxes(false)
                //.setDrawCubeProjection(false)
                //.setDrawTagOutline(true)
//                .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                .setTagLibrary(AprilTagGameDatabase.getIntoTheDeepTagLibrary())
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                .setCameraPose(cameraPosition, cameraOrientation)

                // == CAMERA CALIBRATION ==
                // If you do not manually specify calibration parameters, the SDK will attempt
                // to load a predefined calibration for your camera.
                //.setLensIntrinsics(578.272, 578.272, 402.145, 221.506)
                // ... these parameters are fx, fy, cx, cy.

                .build();
        // Decimation = 1 ..  Detect 2" Tag from 10 feet away at 10 Frames per second
        // Decimation = 2 ..  Detect 2" Tag from 6  feet away at 22 Frames per second
        // Decimation = 3 ..  Detect 2" Tag from 4  feet away at 30 Frames Per Second (default)
        // Decimation = 3 ..  Detect 5" Tag from 10 feet away at 30 Frames Per Second (default)
        aprilTag.setDecimation(3);

        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder();

        builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));

        // Choose a camera resolution. Not all cameras support all resolutions.
        //builder.setCameraResolution(new Size(640, 480));

        // Enable the RC preview (LiveView).  Set "false" to omit camera monitoring.
        //builder.enableLiveView(true);

        // Set the stream format; MJPEG uses less bandwidth than default YUY2.
        //builder.setStreamFormat(VisionPortal.StreamFormat.YUY2);

        // Choose whether or not LiveView stops if no processors are enabled.
        // If set "true", monitor shows solid orange screen if no processors enabled.
        // If set "false", monitor shows camera view without annotations.
        //builder.setAutoStopLiveView(false);

        // Set and enable the processor.
        builder.addProcessor(aprilTag);

        // Build the Vision Portal, using the above settings.
        visionPortal = builder.build();

        // Disable or re-enable the aprilTag processor at any time.
        visionPortal.setProcessorEnabled(aprilTag, true);
    }

    public Pose2d getRobotPositionFromAprilTags(SampleMecanumDrive drive) {
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        if (currentDetections.size() > 0) {
            // get the first detection
            AprilTagDetection detection = currentDetections.get(0);
            return new Pose2d(detection.robotPose.getPosition().x, detection.robotPose.getPosition().y);
        }
        else {
            telemetry.addData("WARNING", "April Tag not found.");
            return drive.getPoseEstimate();
        }
    }

    public void telemetryAprilTag() {

        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        telemetry.addData("# AprilTags Detected", currentDetections.size());

        // Step through the list of detections and display info for each one.
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)",
                        detection.robotPose.getPosition().x,
                        detection.robotPose.getPosition().y,
                        detection.robotPose.getPosition().z));
                telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)",
                        detection.robotPose.getOrientation().getPitch(AngleUnit.DEGREES),
                        detection.robotPose.getOrientation().getRoll(AngleUnit.DEGREES),
                        detection.robotPose.getOrientation().getYaw(AngleUnit.DEGREES)));
            } else {
                telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
            }
        }   // end for() loop

    }
}
