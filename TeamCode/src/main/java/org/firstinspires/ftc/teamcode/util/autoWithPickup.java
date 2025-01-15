package org.firstinspires.ftc.teamcode.util;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous (name = "Encoderpickup")
public class autoWithPickup extends LinearOpMode {
    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backRight;
    private DcMotor backLeft;
    private Servo outtakePositionLeft;
    private Servo outtakePositionRight;
    private DcMotor motorLeftVert;
    private DcMotor motorRightVert;
    private IMU imu = null;
    private Servo intakeClawServo;
    private Servo outtakeClawServo;
    private Servo horizontalSlideLeft;
    private Servo horizontalSlideRight;
    private Servo passoverServoLeft;
    private Servo passoverServoRight;
    private Servo outtakePositionServoLeft;
    private Servo outtakePositionServoRight;

    double ticksPerRotation = 537.6;
    double wheelCircumference = 11.8737337;                    //11.8737374
    double spoolCircumference = 4.40945;
    double newTarget;
    double midBinHeightInches = -16;
    double topBinHeightInches = -28;
    double midHangInches = -20;
    double topHangInches = -36;
    double topChamber = -22;

    // TODO: SWITCH BOTH FRONT MOTOR ENCODER TARGETS

    @Override
    public void runOpMode() {
        frontLeft = hardwareMap.get(DcMotor.class, "motor0");
        backLeft = hardwareMap.get(DcMotor.class, "motor2");
        frontRight = hardwareMap.get(DcMotor.class, "motor1");
        backRight = hardwareMap.get(DcMotor.class, "motor3");
        outtakePositionLeft= hardwareMap.get(Servo.class,"servo4");
        outtakePositionRight = hardwareMap.get(Servo.class,"servo5");

        motorLeftVert = hardwareMap.get(DcMotor.class, "exMotor0");
        motorRightVert = hardwareMap.get(DcMotor.class, "exMotor1");

        intakeClawServo = hardwareMap.get(Servo.class, "exServo0");
        outtakeClawServo = hardwareMap.get(Servo.class, "servo1");

        horizontalSlideLeft = hardwareMap.get(Servo.class, "servo2");
        horizontalSlideRight = hardwareMap.get(Servo.class, "servo3");

        passoverServoLeft = hardwareMap.get(Servo.class, "exPassoverServo4");
        passoverServoRight = hardwareMap.get(Servo.class,"exPassoverServo5");

        outtakePositionServoLeft = hardwareMap.get(Servo.class,"servo4");
        outtakePositionServoRight = hardwareMap.get(Servo.class,"servo5");

        motorLeftVert.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorRightVert.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // set correct motor direction
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        outtakePositionServoLeft.setDirection(Servo.Direction.REVERSE);
        motorRightVert.setDirection(DcMotorSimple.Direction.REVERSE);

        outtakePositionLeft.setDirection(Servo.Direction.REVERSE);

        waitForStart();

        horizontalSlideLeft.setPosition(0.2);
        horizontalSlideRight.setPosition(0.2);

        forward(7);
        strafeLeft(49.32);
        turnRight(wheelCircumference); // idk change this shit later i cant do this rn

        positionTopOuttake(); // also moves pivot

        forward(-4.5);
        // open claw
        outtakeClawServo.setPosition(0.35);

        // retract outtake pivot
        outtakePositionServoLeft.setPosition(0.15);
        outtakePositionServoRight.setPosition(0.15);

        resetOuttakeSlides();

        turnLeft(wheelCircumference);

        horizontalSlideRight.setPosition(1);
        horizontalSlideLeft.setPosition(1);
        passoverServoLeft.setPosition(0);
        passoverServoRight.setPosition(0);
        intakeClawServo.setPosition(.35);

        strafeRight(3);

        forward(5);
        intakeClawServo.setPosition(.15);
        forward(-5);

        horizontalSlideRight.setPosition(.2);
        horizontalSlideLeft.setPosition(.2);
        passoverServoLeft.setPosition(0.8);
        passoverServoRight.setPosition(0.8);

        strafeRight(-3);
        outtakePositionServoLeft.setPosition(.15);
        outtakePositionServoRight.setPosition(.15);
        turnRight(wheelCircumference);
        outtakeClawServo.setPosition(0.15);
        intakeClawServo.setPosition(0.35);
        forward(-4.5);

        positionTopOuttake();
        outtakeClawServo.setPosition(0.35);

        strafeRight(130);
        forward(-10);


        /*
        servo set power
        sleep(5000)
        move bot
         */
    }
    private double ticksToInchesSpool(double inches) {
        return inches * ticksPerRotation / spoolCircumference;
    }
    private double inchesToTicksWheel(double inches) {
        // ALL NEGATIVE FOR SOME REASON
        return inches * ticksPerRotation / wheelCircumference;
    }

    private boolean isBusy(DcMotor motor) {
        return Math.abs(motor.getCurrentPosition()) >= Math.abs(motor.getTargetPosition());
    }

    public void horizontalExstension(double target){
        newTarget = target;
        horizontalSlideRight.setPosition(target);
        horizontalSlideLeft.setPosition(target);
    }

    public void forward(double target){
        newTarget = inchesToTicksWheel(target);

        frontLeft.setTargetPosition((int)newTarget); // FLIP SIGN CUZ BUG
        frontRight.setTargetPosition((int)newTarget);
        backLeft.setTargetPosition((int)newTarget);
        backRight.setTargetPosition((int)newTarget);

        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // FLIP POWER AS WELL
        frontLeft.setPower(0.3);
        frontRight.setPower(0.3);
        backLeft.setPower(0.3);
        backRight.setPower(0.3);

        while (frontLeft.isBusy() && frontRight.isBusy() && backLeft.isBusy() && backRight.isBusy()) { // add the rest later
//            telemetry.addData("front left target position", frontLeft.getTargetPosition());
//            telemetry.addData("front left position", frontLeft.getCurrentPosition());
//            telemetry.addData("meets target", isBusy(frontLeft));
//            telemetry.addData("front right target position", frontRight.getTargetPosition());
//            telemetry.addData("front right position", frontRight.getCurrentPosition());
//            telemetry.addData("meets target", isBusy(frontRight));
//            telemetry.addData("back left target position", backLeft.getTargetPosition());
//            telemetry.addData("back left position", backLeft.getCurrentPosition());
//            telemetry.addData("meets target", isBusy(backLeft));
//            telemetry.addData("back right target position", backRight.getTargetPosition());
//            telemetry.addData("back right position", backRight.getCurrentPosition());
//            telemetry.addData("meets target", isBusy(backRight));
//            telemetry.update();
        }

        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backLeft.setPower(0);

        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void strafeLeft(double target) {
        newTarget = inchesToTicksWheel(target);
        frontLeft.setTargetPosition((int)-newTarget);
        frontRight.setTargetPosition((int)newTarget);
        backLeft.setTargetPosition((int)newTarget);
        backRight.setTargetPosition((int)-newTarget);

        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        frontLeft.setPower(0.7);
        frontRight.setPower(0.7);
        backLeft.setPower(0.7);
        backRight.setPower(0.7);

        while (frontLeft.isBusy() && frontRight.isBusy() && backLeft.isBusy() && backRight.isBusy()) {}

        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backLeft.setPower(0);

        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void strafeRight(double target) {
        newTarget = inchesToTicksWheel(target);

        frontLeft.setTargetPosition((int)newTarget);
        frontRight.setTargetPosition((int)-newTarget);
        backLeft.setTargetPosition((int)-newTarget);
        backRight.setTargetPosition((int)newTarget);

        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        frontLeft.setPower(0.7);
        frontRight.setPower(0.7);
        backLeft.setPower(0.7);
        backRight.setPower(0.7);

        while (frontLeft.isBusy() && frontRight.isBusy() && backLeft.isBusy() && backRight.isBusy()) {}

        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backLeft.setPower(0);

        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void turnRight(double target) {
        newTarget = inchesToTicksWheel(target);

        telemetry.addData("target", target);
        telemetry.addData("ticks target", newTarget);
        telemetry.update();

        frontLeft.setTargetPosition((int)newTarget);
        frontRight.setTargetPosition((int)-newTarget);
        backLeft.setTargetPosition((int)newTarget);
        backRight.setTargetPosition((int)-newTarget);

        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        frontLeft.setPower(0.7);
        frontRight.setPower(0.7);
        backLeft.setPower(0.7);
        backRight.setPower(0.7);

        while (frontLeft.isBusy() && frontRight.isBusy() && backLeft.isBusy() && backRight.isBusy()) {}

        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backLeft.setPower(0);

        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void turnLeft(double targetAngle){
        double target = targetAngle/360 * wheelCircumference;

        newTarget = inchesToTicksWheel(target);
        frontLeft.setTargetPosition((int)-newTarget);
        frontRight.setTargetPosition((int)newTarget);
        backLeft.setTargetPosition((int)-newTarget);
        backRight.setTargetPosition((int)newTarget);

        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        frontLeft.setPower(0.7);
        frontRight.setPower(0.7);
        backLeft.setPower(0.7);
        backRight.setPower(0.7);

        while (frontLeft.isBusy() && frontRight.isBusy() && backLeft.isBusy() && backRight.isBusy()) {}

        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backLeft.setPower(0);

        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void positionTopOuttake() {
        newTarget = ticksToInchesSpool(topBinHeightInches);

        motorLeftVert.setTargetPosition((int) newTarget);
        motorRightVert.setTargetPosition((int) newTarget);

        motorLeftVert.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorRightVert.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        motorLeftVert.setPower(0.7);
        motorRightVert.setPower(0.7);

        outtakePositionServoLeft.setPosition(0.75);
        outtakePositionServoRight.setPosition(0.75);

        while (motorLeftVert.isBusy() && motorRightVert.isBusy()) {}
    }

    public void resetOuttakeSlides() {
        motorLeftVert.setTargetPosition(0);
        motorRightVert.setTargetPosition(0);

        motorLeftVert.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorRightVert.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        motorLeftVert.setPower(0.5);
        motorRightVert.setPower(0.5);
    }
}
