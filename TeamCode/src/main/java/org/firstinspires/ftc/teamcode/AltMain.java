package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

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

@TeleOp(name="Alt Main", group="Linear OpMode")
public class AltMain extends LinearOpMode {

    // Declare OpMode members for each of the 4 motors.
    /*
    motor 0  = Front_left
    Motor1  = Back_Left
    Motor2 = Front_Right
    Motor3 = Back_Right
     */
    //these are variables we declare what electronics we're using
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor frontLeftDrive = null;
    private DcMotor backLeftDrive = null;
    private DcMotor frontRightDrive = null;
    private DcMotor backRightDrive = null;
    private IMU imu = null;
    private Servo intakeClawServo;
    private Servo outtakeClawServo;
    private Servo horizontalSlideLeft;
    private Servo horizontalSlideRight;
    private Servo passoverServoLeft;
    private Servo passoverServoRight;

    private DcMotor motorLeftVert;
    private DcMotor motorRightVert;
    private Servo outtakePositionServoLeft;
    private Servo outtakePositionServoRight;
    private DcMotor motorHang;
    //these variables are mostly values we give values to or will give values to.
    double ticksPerRotation = 537.6;
    double wheelCircumference = 11.8737374;
    double spoolCircumference = 4.40945;
    double goBuildaMaxServoAngle = 300;
    double servoTarget;
    double newTarget;
    //These are negatives due to how the motors are facing
    double midBinHeightInches = -16;
    double topBinHeightInches = -38;
    double midHangInches = -20;
    double topHangInches = -36;
    double topChamber = -22;
    //toggleSwitch between Field Centric Drive to Robot Centric Drive
    boolean toggleSwitch = false;

    @Override
    public void runOpMode() throws InterruptedException {
        //if the device name you'll get an error from the driver hub tell you what's wrong
        frontLeftDrive = hardwareMap.get(DcMotor.class, "motor0");
        backLeftDrive = hardwareMap.get(DcMotor.class, "motor1");
        frontRightDrive = hardwareMap.get(DcMotor.class, "motor2");
        backRightDrive = hardwareMap.get(DcMotor.class, "motor3");

        // todo:change these as wiring has changed
        intakeClawServo = hardwareMap.get(Servo.class, "exServo0");
        outtakeClawServo = hardwareMap.get(Servo.class, "servo0");

        horizontalSlideLeft = hardwareMap.get(Servo.class, "servo2");
        horizontalSlideRight = hardwareMap.get(Servo.class, "servo3");

        passoverServoLeft = hardwareMap.get(Servo.class, "exPassoverServo4");
        passoverServoRight = hardwareMap.get(Servo.class,"exPassoverServo5");

        outtakePositionServoLeft = hardwareMap.get(Servo.class,"servo4");
        outtakePositionServoRight = hardwareMap.get(Servo.class,"servo5");



        motorLeftVert = hardwareMap.get(DcMotor.class, "exMotor0");
        motorRightVert = hardwareMap.get(DcMotor.class, "exMotor1");
        motorHang = hardwareMap.get(DcMotor.class, "exMotor2");

        //because we often have motors/servos in pairs facing opposite ways we have to reverse directions
        // so they turn in the same direction
        horizontalSlideRight.setDirection(Servo.Direction.REVERSE); // this ain't a typo it supposed to be right idk why
        passoverServoLeft.setDirection(Servo.Direction.REVERSE);
        motorRightVert.setDirection(DcMotorSimple.Direction.REVERSE);
        outtakePositionServoLeft.setDirection(Servo.Direction.REVERSE);
        // reset encoders
        motorLeftVert.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRightVert.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorHang.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // set correct motor direction
        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        backRightDrive.setDirection(DcMotor.Direction.FORWARD);

        //this is a thing in the Control Hub which determines the direction of the robot
        imu = hardwareMap.get(IMU.class, "imu");
        //these determines the direction of the robot depending on how the control hub is oriented
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

        while (opModeIsActive()) {
            double max;
            // don't question it. I don't know why it works like this. It just does.
            double x = -gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
            double y = gamepad1.left_stick_x; //x
            double rx = gamepad1.right_stick_x; //rx

            //will reset the direction of the robot relative of the position it is currently at
            if (gamepad1.options) {
                imu.resetYaw();
            }

            double heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            //does math stuff to keep directions constant
            double adjustedX = -y * Math.sin(heading) + x * Math.cos(heading);
            double adjustedY =  y * Math.cos(heading) + x * Math.sin(heading);

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

            // todo: try to automate the passover more, aka less button presses

            // MAX DEGREES IS 0 TO 300.

            //close intake and open outtake
            if (gamepad2.right_trigger > 0) {
                outtakeClawServo.setPosition(0.4);
                intakeClawServo.setPosition(0.15);
            }

//            if (gamepad2.right_trigger > 0) {
//                outtakeClawServo.setPosition(servoAngle(120));
//                intakeClawServo.setPosition(servoAngle(45));
//            }

                //open intake and close outtake
                if (gamepad2.left_trigger > 0) {
                    outtakeClawServo.setPosition(0.15);
                    intakeClawServo.setPosition(0.4);
                }
//
//            if (gamepad2.left_trigger > 0) {
//                outtakeClawServo.setPosition(servoAngle(servoAngle(45)));
//                intakeClawServo.setPosition(servoAngle(120));
//            }

            // extend
            if (gamepad2.a) {
                //SUBJECT TO CHANGE DUE TO MECHANICAL AND SERVO RANGE
                //also set servo range to the specific rotation we need
                horizontalSlideRight.setPosition(0.5);
                horizontalSlideLeft.setPosition(0.5);
                passoverServoLeft.setPosition(0.1);
                passoverServoRight.setPosition(0.1);
            }

            // midway point
            if (gamepad1.a) {
                horizontalSlideRight.setPosition(0.4);
                horizontalSlideLeft.setPosition(0.4);
            }

            // retract
            if (gamepad2.b) {
                horizontalSlideLeft.setPosition(0.3);
                horizontalSlideRight.setPosition(0.3);
                passoverServoRight.setPosition(0.8);
                passoverServoLeft.setPosition(0.8);
            }

            if (gamepad2.x) {
                outtakePositionServoLeft.setPosition(0.1);
                outtakePositionServoRight.setPosition(0.1);
            }
            if (gamepad2.y) {
                outtakePositionServoLeft.setPosition(0.75);
                outtakePositionServoRight.setPosition(0.75);
            }

            //grabs specimen from wall
            if (gamepad2.right_bumper){
                outtakePositionServoRight.setPosition(0.95);
                outtakePositionServoLeft.setPosition(0.95);
            }



            // todo:tune these positions since slides have changed height
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

            if(gamepad2.dpad_left){
                topChamber();
            }

            /* This gives us data un parts of the robot we want
            A template to do this is:
            telementry.addDate("What you want the name of the data to show up as ", [mechanism you want to check].getCurrentPosition());
            Now some parts can very depending what you want
            Now for example, you can get the power of a motor or continuous servo by using '.getCurrentPower'
                 */
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
            telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);
            telemetry.addData("Motor ticks 1", motorLeftVert.getCurrentPosition());
            telemetry.addData("Motor ticks 2", motorRightVert.getCurrentPosition());
//            telemetry.addData("Servo position", servoAngle(servoTarget));
            telemetry.update();
        }
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
    private double ticksToInchesSpool(double inches) {
        return inches * ticksPerRotation / spoolCircumference;
    }

    private double ticksToInchesWheel(double inches) {
        return inches * ticksPerRotation / wheelCircumference;
    }
    //todo: integrate this into servo positions
    private double servoAngle(double degree){
      servoTarget = degree/goBuildaMaxServoAngle;
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

        motorLeftVert.setPower(0.8);
        motorRightVert.setPower(0.7);
    }

    public void positionMidOuttake() {
        newTarget = ticksToInchesSpool(midBinHeightInches);

        motorLeftVert.setTargetPosition((int) newTarget);
        motorRightVert.setTargetPosition((int) newTarget);

        motorLeftVert.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorRightVert.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        motorLeftVert.setPower(0.8);
        motorRightVert.setPower(0.8);
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
        motorRightVert.setTargetPosition((int)newTarget);
        motorLeftVert.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorRightVert.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorRightVert.setPower(0.4);
        motorLeftVert.setPower(0.5);
    }

    //reset position
    public void resetOuttakeSlides() {
        motorLeftVert.setTargetPosition(0);
        motorRightVert.setTargetPosition(0);

        motorLeftVert.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorRightVert.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        motorLeftVert.setPower(0.5);
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
