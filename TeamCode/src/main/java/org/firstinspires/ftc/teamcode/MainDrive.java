package org.firstinspires.ftc.teamcode;
/*
notes:
make complete automation during teleop, back and forth
 */

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name="Main Drive")
public class MainDrive extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor frontLeftDrive;
    private DcMotor backLeftDrive;
    private DcMotor backRightDrive;
    private DcMotor frontRightDrive;
    private IMU imu = null;
//    private CRServo intakeServoLeft;
//    private CRServo intakeServoRight;
//    private Servo horizontalSlideLeft;
//    private Servo horizontalSlideRight;
//    private Servo passoverServoLeft;
//    private Servo passoverServoRight;
//    private DcMotor motorLeftVert;
//    private DcMotor motorRightVert;
//    private Servo outtakeServoLeft;
//    private Servo outtakeServoRight;
//    private Servo gateServo;
//    private DcMotor motorHang;


//    //toggleSwitch between Field Centric Drive to Robot Centric Drive
//    boolean toggleSwitch = false;

    @Override
    public void runOpMode() {
        frontLeftDrive = hardwareMap.get(DcMotor.class, "motor0");
        backLeftDrive = hardwareMap.get(DcMotor.class, "motor1");
        frontRightDrive = hardwareMap.get(DcMotor.class, "motor2");
        backRightDrive = hardwareMap.get(DcMotor.class, "motor3");

        /* The device name stands for which motor/servo port it correlates to. ex stands for
        expansion hub
        For example: motor 0 is the motor port labeled zero on control hub
        exServo 0 is the servo port labeled 0 on the expansion hub
        */
//        motorLeftVert = hardwareMap.get(DcMotor.class, "exMotor0");
//        motorLeftVert.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        motorRightVert = hardwareMap.get(DcMotor.class, "exMotor1");
//        motorRightVert.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        motorHang = hardwareMap.get(DcMotor.class, "exMotor2");
//        motorHang.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//
//        intakeServoLeft = hardwareMap.get(CRServo.class, "servo1");
//        intakeServoRight = hardwareMap.get(CRServo.class, "servo0");
//        horizontalSlideLeft = hardwareMap.get(Servo.class, "servo2");
//        horizontalSlideRight = hardwareMap.get(Servo.class, "servo3");
//        passoverServoLeft = hardwareMap.get(Servo.class, "passoverServo4");
//        passoverServoRight = hardwareMap.get(Servo.class,"passoverServo5");
//        outtakeServoLeft = hardwareMap.get(Servo.class,"exServo0");
//        outtakeServoRight = hardwareMap.get(Servo.class,"exServo1");
//        gateServo = hardwareMap.get(Servo.class,"exServo2");

        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backRightDrive.setDirection(DcMotor.Direction.FORWARD);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
//
//        intakeServoLeft.setDirection(CRServo.Direction.REVERSE);
//        horizontalSlideRight.setDirection(Servo.Direction.REVERSE); // this ain't a typo it supposed to be right idk why
//        passoverServoLeft.setDirection(Servo.Direction.REVERSE);
//        motorLeftVert.setDirection(DcMotor.Direction.REVERSE);
//        motorHang.setDirection(DcMotorSimple.Direction.REVERSE);
//        outtakeServoLeft.setDirection(Servo.Direction.REVERSE);


        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(IMU.class, "imu");

        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
        ));

        imu.initialize(parameters);

        // Wait for the game to start (driver presses START)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            double max;

            // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
            double y = -gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
            double x = gamepad1.left_stick_x; //x
            double rx = gamepad1.right_stick_x; //rx

            //removed negative from here
            if (gamepad1.options) {
                imu.resetYaw();
            }

            double heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            double adjustedX = x * Math.cos(-heading) - y * Math.sin(-heading);
            double adjustedY = x * Math.sin(-heading) + y * Math.cos(-heading);

            // Combine the joystick requests for each axis-motion to determine each wheel's power.
//            // Set up a variable for each drive wheel to save the power level for telemetry.

            double leftFrontPower = adjustedY + adjustedX + rx;
            double rightFrontPower = adjustedY - adjustedX - rx;
            double leftBackPower = adjustedY - adjustedX + rx;
            double rightBackPower = adjustedY + adjustedX - rx;

            // Normalize the values so no wheel power exceeds 100%
            // This ensures that the robot maintains the desired motion.
            max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
            max = Math.max(max, Math.abs(leftBackPower));
            max = Math.max(max, Math.abs(rightBackPower));
            //sets default speed
            double power = 0.7;
            //slow mode
            if (gamepad1.right_bumper) {
                power = (power * 0.5);
            }
            //fast mode
            if (gamepad1.left_bumper) {
                power = 1;
            }

//            if (max > 1.0) {
//                leftFrontPower /= max;
//                rightFrontPower /= max;
//                leftBackPower /= max;
//                rightBackPower /= max;
//            }

//            if (gamepad2.options && !toggleSwitch) {
//                toggleSwitch = true;
//            } else if (gamepad2.options && toggleSwitch) {
//                toggleSwitch = false;
//            }
//            //converts from field oriented drive to robot oriented drive
//            if (toggleSwitch) {
//                leftFrontPower = y + x + rx;
//                rightFrontPower = y - x - rx;
//                leftBackPower = y - x + rx;
//                rightBackPower = y + x - rx;
//            }
//
            frontLeftDrive.setPower(leftFrontPower * power);
            backRightDrive.setPower(rightFrontPower * power);
            backLeftDrive.setPower(leftBackPower * power);
            frontRightDrive.setPower(rightBackPower * power);
//
//            //Intake System
//            if (gamepad1.right_trigger > 0 || gamepad2.right_trigger > 0) {
//                intakeServoLeft.setPower(-0.5);
//                intakeServoRight.setPower(-0.5);
//            } else if (gamepad1.left_trigger > 0) {
//                    intakeServoLeft.setPower(0.5);
//                    intakeServoRight.setPower(0.5);
//                }
//                else {
//                    intakeServoLeft.setPower(0);
//                    intakeServoRight.setPower(0);
//                }
//
//            if (gamepad1.a) {
//                //SUBJECT TO CHANGE DUE TO MECHANICAL AND SERVO RANGE
//                //also set servo range to the specific rotation we need
//                horizontalSlideRight.setPosition(0.5);
//                horizontalSlideLeft.setPosition(0.5);
//            }


            /*
       Should make this a sequence from returning to reset position to passing to outtake so we
       we don't need to press an input for each individual part
       */
            // retract
//            if (gamepad1.b) {
//                horizontalSlideLeft.setPosition(0.3);
//                horizontalSlideRight.setPosition(0.3);
//            }
//            //intake position
//            if (gamepad1.x){
//                passoverServoLeft.setPosition(0.4);
//                passoverServoRight.setPosition(0.4);
//            }
//            //passover position
//            if (gamepad1.y){
//                passoverServoRight.setPosition(0.9);
//                passoverServoLeft.setPosition(0.9);
//            }
//            //The intake has a gate that prevents it from spitting the sample from it's backside.
//            if (gamepad2.left_bumper){
//                gateServo.setPosition(1);
//            } else {
//                gateServo.setPosition(0);
//            }
//
//            //outtake system
//            if (gamepad2.dpad_up) {
//                //telling the motor to rotate one inch
//                positionTopOuttake();
//            }
//
//            if (gamepad2.dpad_right){
//                positionMidOuttake();
//            }
//            //continues to keep track of ticks after pressing the button, it'll keep the position until a separate input tells it otherwise.
//
//            //bring it back one inch, this will also hold the slides in the same position.
//            if (gamepad2.dpad_down) {
//                resetOuttakeSlides();
//            }
//
//            //hang commands
//            if (gamepad2.y) {
//                positionTopHang();
//            }
//            if (gamepad2.a){
//                resetHang();
//            }
//            if (gamepad2.x) {
//                positionMidHang();
//            }
//
//            //Outtake servo code here
//            if (gamepad2.left_trigger > 0){
//                outtakeServoLeft.setPosition(1);
//                outtakeServoRight.setPosition(1);
//            } else {
//                outtakeServoLeft.setPosition(0);
//                outtakeServoRight.setPosition(0);
//            }

//            if (gamepad2.left_trigger > 0){
//                outtakeServoLeft.setPosition(1);
//                outtakeServoRight.setPosition(1);
//            }

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
            telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);
//            telemetry.addData("Horizontal Slide Left Position",horizontalSlideLeft.getPosition());
//            telemetry.addData("Horizontal Slide Right Position",horizontalSlideRight.getPosition());
//            telemetry.addData("Passover Slide Left Position",passoverServoLeft.getPosition());
//            telemetry.addData("Passover Slide Right Position",passoverServoRight.getPosition());
            telemetry.update();
        }
    }

    //positions and conversions

    double ticksPerRotation = 537.6;
    double wheelCircumference = 11.8737374;
    double spoolCircumference = 4.40945;
    //finds the amount of ticks per inch of rotation
    // 2786.2 ticks /1.4 inch = 1990 ticks per inch of rotation
    double newTarget;
    double midBinHeightInches = -16;
    double topBinHeightInches = -43;
    double midHangInches = -20;
    double topHangInches = -36;

    private double ticksToInchesSpool(double inches) {
        return inches * ticksPerRotation / spoolCircumference;
    }

//    public void positionTopOuttake() {
////        motorLeftVert.setDirection(DcMotor.Direction.REVERSE);
////        motorLeftVert.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
////        motorRightVert.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        newTarget = ticksToInchesSpool(topBinHeightInches);
//
//        motorLeftVert.setTargetPosition((int) newTarget);
//        motorRightVert.setTargetPosition((int) newTarget);
//
//        motorLeftVert.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        motorRightVert.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//        motorLeftVert.setPower(0.8);
//        motorRightVert.setPower(0.8);
//    }
//    private double ticksToInchesWheel(double inches) {
//        return inches * ticksPerRotation / wheelCircumference;
//    }
//
//    public void positionMidOuttake() {
////        motorLeftVert.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
////        motorRightVert.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        newTarget = ticksToInchesSpool(midBinHeightInches);
//
//        motorLeftVert.setTargetPosition((int) newTarget);
//        motorRightVert.setTargetPosition((int) newTarget);
//
//        motorLeftVert.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        motorRightVert.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//        motorLeftVert.setPower(0.8);
//        motorRightVert.setPower(0.8);
//    }
//
//    public void positionMidHang() {
////        motorHang.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        newTarget = ticksToInchesSpool(midHangInches);
//        motorHang.setTargetPosition((int) newTarget);
//        motorHang.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        motorHang.setPower(0.4);
//    }

//    public void positionTopHang() {
////        motorHang.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        newTarget = ticksToInchesSpool(topHangInches);
//        motorHang.setTargetPosition((int) newTarget);
//        motorHang.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        motorHang.setPower(0.4);
//    }
//
//    //reset position
//    public void resetOuttakeSlides() {
//        motorLeftVert.setTargetPosition(0);
//        motorRightVert.setTargetPosition(0);
//
//        motorLeftVert.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        motorRightVert.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//        motorLeftVert.setPower(0.5);
//        motorRightVert.setPower(0.5);
//    }
//    public void resetHang() {
////       motorHang.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        /*wheelRevolution = 96mm = 3.78 inches if for wheels
//          wheelRevolution will equal inner diameter of spool if for linear spool */
//        motorHang.setTargetPosition(0);
//        motorHang.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        motorHang.setPower(0.6);
//    }
}
