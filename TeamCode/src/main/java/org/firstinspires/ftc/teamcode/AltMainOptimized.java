package org.firstinspires.ftc.teamcode;


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

/*
 * This file contains an example of a Linear "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When a selection is made from the menu, the corresponding OpMode is executed.
 *
 * This particular OpMode illustrates driving a 4-motor Omni-Directional (or Holonomic) robot.
 * This code will work with either a Mecanum-Drive or an X-Drive train.
 * Both of these drives are illustrated at https://gm0.org/en/latest/docs/robot-design/drivetrains/holonomic.html
 * Note that a Mecanum drive must display an X roller-pattern when viewed from above.
 *
 * Also note that it is critical to set the correct rotation direction for each motor.  See details below.
 *
 * Holonomic drives provide the ability for the robot to move in three axes (directions) simultaneously.
 * Each motion axis is controlled by one Joystick axis.
 *
 * 1) Axial:    Driving forward and backward               Left-joystick Forward/Backward
 * 2) Lateral:  Strafing right and left                     Left-joystick Right and Left
 * 3) Yaw:      Rotating Clockwise and counter clockwise    Right-joystick Right and Left
 *
 * This code is written assuming that the right-side motors need to be reversed for the robot to drive forward.
 * When you first test your robot, if it moves backward when you push the left stick forward, then you must flip
 * the direction of all 4 motors (see code below).
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@TeleOp(name="Optimized Main", group="Linear OpMode")
public class AltMainOptimized extends LinearOpMode {

    // Declare OpMode members for each of the 4 motors.
    /*
    motor 0  = Front_left
    Motor1  = Back_Left
    Motor2 = Front_Right
    Motor3 = Back_Right
     */
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor frontLeftDrive = null;
    private DcMotor backLeftDrive = null;
    private DcMotor frontRightDrive = null;
    private DcMotor backRightDrive = null;
    private IMU imu = null;
    private CRServo intakeServoLeft;
    private CRServo intakeServoRight;
    private Servo horizontalSlideLeft;
    private Servo horizontalSlideRight;
    private Servo passoverServoLeft;
    private Servo passoverServoRight;

    private DcMotor motorLeftVert;
    private DcMotor motorRightVert;
    private CRServo outtakeServoLeft;
    private CRServo outtakeServoRight;
    private Servo gateServo;

    private DcMotor motorHang;
    double ticksPerRotation = 537.6;
    double wheelCircumference = 11.8737374;
    double spoolCircumference = 4.40945;

    double newTarget;
    double midBinHeightInches = 16;
    double topBinHeightInches = 43;
    double midHangInches = 20;
    double topHangInches = 36;

    //toggleSwitch between Field Centric Drive to Robot Centric Drive
    boolean toggleSwitch = false;

    @Override
    public void runOpMode() {
        frontLeftDrive = hardwareMap.get(DcMotor.class, "motor0");
        backLeftDrive = hardwareMap.get(DcMotor.class, "motor1");
        frontRightDrive = hardwareMap.get(DcMotor.class, "motor2");
        backRightDrive = hardwareMap.get(DcMotor.class, "motor3");

        intakeServoLeft = hardwareMap.get(CRServo.class, "servo0");
        intakeServoRight = hardwareMap.get(CRServo.class, "servo1");
        horizontalSlideLeft = hardwareMap.get(Servo.class, "servo2");
        horizontalSlideRight = hardwareMap.get(Servo.class, "servo3");
        passoverServoLeft = hardwareMap.get(Servo.class, "exPassoverServo3");
        passoverServoRight = hardwareMap.get(Servo.class,"passoverServo5"); // expansion hub
        motorLeftVert = hardwareMap.get(DcMotor.class, "exMotor0");
        motorRightVert = hardwareMap.get(DcMotor.class, "exMotor1");
        motorHang = hardwareMap.get(DcMotor.class, "exMotor2");
        outtakeServoLeft = hardwareMap.get(CRServo.class,"exServo0");
        outtakeServoRight = hardwareMap.get(CRServo.class,"exServo1");
        gateServo = hardwareMap.get(Servo.class,"exServo2");

        intakeServoLeft.setDirection(CRServo.Direction.REVERSE);
        horizontalSlideRight.setDirection(Servo.Direction.REVERSE); // this aint a typo it supposed to be right idk why
        passoverServoLeft.setDirection(Servo.Direction.REVERSE);
        motorRightVert.setDirection(DcMotorSimple.Direction.REVERSE);
        outtakeServoLeft.setDirection(CRServo.Direction.REVERSE);
        // reset encoders
        motorLeftVert.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRightVert.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorHang.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // set correct motor direction
        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        backRightDrive.setDirection(DcMotor.Direction.FORWARD);

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

//        int target = (int) ticksToInchesSpool(0.2);
//        if (target == 0) target = 1;
//        motorLeftVert.setTargetPosition(target);
//        motorLeftVert.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        motorLeftVert.setPower(0.2);
//
//        while (motorLeftVert.isBusy()) {} // TODO: i shouldnt be stalling this, but i have to right now. fix it later
//
//        motorLeftVert.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        motorLeftVert.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

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

            //If it doesn't work properly remove the negative from the Axial
            //Removed nagative from axial(8:10 AM, 9/30/24)
            //You may need to switch Axial to Lateral.
            double adjustedX = x * Math.cos(-heading) - y * Math.sin(-heading);
            double adjustedY = x * Math.sin(-heading) + y * Math.cos(-heading);

            // Combine the joystick requests for each axis-motion to determine each wheel's power.
            // Set up a variable for each drive wheel to save the power level for telemetry.
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
            double default_power = 0.7;
            double power = default_power;
            //slow mode
            if (gamepad1.right_bumper) {
                power = default_power * 0.5;
            }

            // fast mode
            if (gamepad1.left_bumper) {
                power = 1;
            }

            if (max > 1.0) {
                leftFrontPower /= max;
                rightFrontPower /= max;
                leftBackPower /= max;
                rightBackPower /= max;
            }

            if (gamepad1.left_bumper && !toggleSwitch) {
                toggleSwitch = true;
            } else if (gamepad1.left_bumper && toggleSwitch) {
                toggleSwitch = false;
            }
            //converts from field oriented drive to robot oriented drive
            if (!gamepad1.left_bumper & toggleSwitch) {
                leftFrontPower = y + x + rx;
                rightFrontPower = y - x - rx;
                leftBackPower = y - x + rx;
                rightBackPower = y + x - rx;
            }

            // drive
            frontLeftDrive.setPower(leftFrontPower * power);
            frontRightDrive.setPower(rightFrontPower * power);
            backLeftDrive.setPower(leftBackPower * power);
            backRightDrive.setPower(rightBackPower * power);



            // todo: non drive section
            if (gamepad2.right_bumper) {
                gateServo.setPosition(0.5);
            }

            if (gamepad2.left_bumper) {
                gateServo.setPosition(0.25);
            }

            //intake
            if (gamepad2.right_trigger > 0) {
                intakeServoLeft.setPower(0.7);
                intakeServoRight.setPower(0.7);
            }
            //outtake
            else if (gamepad2.left_trigger > 0) {
                intakeServoLeft.setPower(-0.7);
                intakeServoRight.setPower(-0.7);
            } else {
                intakeServoLeft.setPower(0);
                intakeServoRight.setPower(0);
            }


            // retract to passover position
            if (gamepad2.y) {
                horizontalSlideLeft.setPosition(0.2);
                horizontalSlideRight.setPosition(0.2);
                passoverServoRight.setPosition(0.26);
                passoverServoLeft.setPosition(0.26);
            }
            //intake position, but no extend with slides (regular y stick's values are inversed)
            if (-gamepad2.right_stick_y < 0){
                passoverServoRight.setPosition(0.69);
                passoverServoLeft.setPosition(0.69);
            }

            // extend to intake position
            if (gamepad2.x) {
                passoverServoLeft.setPosition(0.69);
                passoverServoRight.setPosition(0.69);
                horizontalSlideRight.setPosition(0.5);
                horizontalSlideLeft.setPosition(0.5);
            }

            // todo:tune these positions when we get working slides
            if (gamepad2.dpad_up) {
                //telling the motor to rotate one inch
                positionTopOuttake();
            }

            if (gamepad2.dpad_right) {
                positionMidOuttake();
            }

            if (gamepad2.dpad_down) {
                resetOuttakeSlides();
            }

//            if (gamepad2.y) {
//                positionTopHang();
//            }
//            if (gamepad2.a) {
//                resetHang();
//            }
//            if (gamepad2.x) {
//                positionMidHang();
//            }

            //ensure the servos are flipped. They should be positioned to have the wires facing down.
            // change this to be Set rotation and not continuous
            if (gamepad2.a) {
                outtakeServoLeft.setPower(0.15);
                outtakeServoRight.setPower(0.15);
            } else if (gamepad2.b) {
                outtakeServoLeft.setPower(-0.15);
                outtakeServoRight.setPower(-0.15);
            } else {
                outtakeServoLeft.setPower(0);
                outtakeServoRight.setPower(0);
            }

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
            telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);
            telemetry.addData("Motor ticks 1", motorLeftVert.getCurrentPosition());
            telemetry.addData("Motor ticks 2", motorRightVert.getCurrentPosition());
            telemetry.addData("Hang Ticks", motorHang.getCurrentPosition());
            telemetry.update();
        }
    }
    //conversions
    private double ticksToInchesSpool(double inches) {
        return inches * ticksPerRotation / spoolCircumference;
    }

    private double ticksToInchesWheel(double inches) {
        return inches * ticksPerRotation / wheelCircumference;
    }

    // todo: lowered speed
    //Tells the robot the position you want it to go to with how much power.
    public void positionTopOuttake() {
//        motorLeftVert.setDirection(DcMotor.Direction.REVERSE);
//        motorLeftVert.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        motorRightVert.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        newTarget = ticksToInchesSpool(topBinHeightInches);

        motorLeftVert.setTargetPosition((int) newTarget);
        motorRightVert.setTargetPosition((int) newTarget);

        motorLeftVert.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorRightVert.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        motorLeftVert.setPower(0.4);
        motorRightVert.setPower(0.4);
    }

    public void positionMidOuttake() {
//        motorLeftVert.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        motorRightVert.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        newTarget = ticksToInchesSpool(midBinHeightInches);

        motorLeftVert.setTargetPosition((int) newTarget);
        motorRightVert.setTargetPosition((int) newTarget);

        motorLeftVert.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorRightVert.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        motorLeftVert.setPower(0.4);
        motorRightVert.setPower(0.4);
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

    //reset position
    public void resetOuttakeSlides() {
        motorLeftVert.setTargetPosition(0);
        motorRightVert.setTargetPosition(0);

        motorLeftVert.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorRightVert.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        motorLeftVert.setPower(0.4);
        motorRightVert.setPower(0.4);
    }
    public void resetHang() {
//       motorHang.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        /*wheelRevolution = 96mm = 3.78 inches if for wheels
          wheelRevolution will equal inner diameter of spool if for linear spool */
        motorHang.setTargetPosition(0);
        motorHang.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorHang.setPower(0.6);
    }

}
