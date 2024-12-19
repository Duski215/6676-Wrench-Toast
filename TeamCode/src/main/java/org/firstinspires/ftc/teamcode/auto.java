package org.firstinspires.ftc.teamcode;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="Robot: Auto Drive By Encoder", group="Robot")
public class auto extends LinearOpMode {

    /* Declare OpMode members. */
    DcMotor frontLeft   = null;
    DcMotor frontRight  = null;
    DcMotor backLeft = null;
    DcMotor backRight = null;

    ElapsedTime runtime = new ElapsedTime();

    // Calculate the COUNTS_PER_INCH for your specific drive train.
    // Go to your motor vendor website to determine your motor's COUNTS_PER_MOTOR_REV
    // For external drive gearing, set DRIVE_GEAR_REDUCTION as needed.
    // For example, use a value of 2.0 for a 12-tooth spur gear driving a 24-tooth spur gear.
    // This is gearing DOWN for less speed and more torque.
    // For gearing UP, use a gear ratio less than 1.0. Note this will affect the direction of wheel rotation.
    final double     COUNTS_PER_MOTOR_REV    = 537.6 ;
    final double     DRIVE_GEAR_REDUCTION    = 1.0 ;
    final double     WHEEL_DIAMETER_INCHES   = 3.78 ;     // For figuring circumference
    final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    final double     DRIVE_SPEED             = 1;
    final double     TURN_SPEED              = 1;

    public void runOpMode() {
        // Initialize the drive system variables.
        frontLeft  = hardwareMap.get(DcMotor.class, "motor0");
        frontRight = hardwareMap.get(DcMotor.class, "motor2");
        backLeft = hardwareMap.get(DcMotor.class, "motor1");
        backRight = hardwareMap.get(DcMotor.class, "motor3");

        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // When run, this OpMode should start both motors driving forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.FORWARD);

        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Starting at",  "%7d :%7d",
                frontLeft.getCurrentPosition(),
                frontRight.getCurrentPosition(),
                backLeft.getCurrentPosition(),
                backRight.getCurrentPosition()
        );
        telemetry.update();

        // Wait for the game to start (driver presses START)
        waitForStart();

        // Step through each leg of the path
        // TODO: PATHING

        //blue close
        moveBackward(12);
        moveForward(12);
        turnRight(90);

        telemetry.addData("Path", "Complete");
        telemetry.update();
        sleep(1000);  // pause to display final telemetry message.
    }

    public void moveForward(double inches) {
        encoderDrive(DRIVE_SPEED, inches, inches);
    }

    public void moveBackward(double inches) {
        encoderDrive(DRIVE_SPEED, -inches, -inches);
    }

    /**
     * degrees, not radians
     * @param degrees
     */
    public void turnRight(double degrees) {
        double inches = degrees * WHEEL_DIAMETER_INCHES * Math.PI / 360;
        encoderDrive(TURN_SPEED, inches, -inches);
    }

    /**
     * degrees, not radians
     * @param degrees
     */
    public void turnLeft(double degrees) {
        double inches = degrees * WHEEL_DIAMETER_INCHES * Math.PI / 360;
        encoderDrive(TURN_SPEED, -inches, inches);
    }

    /*
     *  Method to perform a relative move, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the OpMode running.
     */
    public void encoderDrive(double speed,
                             double leftInches, double rightInches
//                             , double timeoutS
    ) {
        // Determine new target position, and pass to motor controller
        int newFrontLeftTarget = (int)(leftInches * COUNTS_PER_INCH);
        int newFrontRightTarget = (int)(rightInches * COUNTS_PER_INCH);
        int newBackLeftTarget = (int)(leftInches * COUNTS_PER_INCH);
        int newBackRightTarget = (int)(rightInches * COUNTS_PER_INCH);

        frontLeft.setTargetPosition(newFrontLeftTarget);
        frontRight.setTargetPosition(newFrontRightTarget);
        backLeft.setTargetPosition(newBackLeftTarget);
        backRight.setTargetPosition(newBackRightTarget);

        // Turn On RUN_TO_POSITION
        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // reset the timeout time and start motion.
        runtime.reset();
        frontLeft.setPower(speed);
        frontRight.setPower(speed);
        backLeft.setPower(speed);
        backRight.setPower(speed);

        // keep looping while we are still active, and there is time left, and both motors are running.
        // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
        // its target position, the motion will stop.  This is "safer" in the event that the robot will
        // always end the motion as soon as possible.
        // However, if you require that BOTH motors have finished their moves before the robot continues
        // onto the next step, use (isBusy() || isBusy()) in the loop test.
        while (opModeIsActive() &&
//                    (runtime.seconds() < timeoutS) &&
                (frontLeft.isBusy() && frontRight.isBusy()) && backLeft.isBusy() && backRight.isBusy()) {

            // Display it for the driver.
            telemetry.addData("Running to",  " %7d :%7d", newFrontLeftTarget,  newFrontRightTarget, newBackLeftTarget, newBackRightTarget);
            telemetry.addData("Currently at",  " at %7d :%7d",
                    frontLeft.getCurrentPosition(), frontRight.getCurrentPosition(), backLeft.getCurrentPosition(), backRight.getCurrentPosition());
            telemetry.update();
        }

        // Stop all motion;
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);

        // reset encoders back to zero position
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        sleep(2000);   // optional pause after each move.
    }
}




