package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

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

@TeleOp(name="Main Drive", group="Linear OpMode")
public class MainDrive extends LinearOpMode {
    public Robot r;
    public Boolean buffer = false;

    private double startTime;
    private double bufferTime = 0.6; // this is variable. tune for better time

    public void startHardstopBuffer() {
        buffer = true;
        startTime = r.runtime.time();
    }

    public void hardstopBufferAction() {
        double time = r.runtime.time();
        if (time - startTime > bufferTime) {// if elapsed time is greater than the buffered time, then close the claw and reset buffer
            // activate the hardstop claw
            r.activateHardStop();

            // reset buffer
            buffer = false;
            startTime = 0; // technically dont need this but im just doing it
        }
    }

    @Override
    public void runOpMode() throws InterruptedException {
        // instantiate our robot object
        r = new Robot(hardwareMap, telemetry);

        waitForStart();

        // reset imu
        r.imu.resetYaw();
        // reset runtime
        r.runtime.reset();

        while (opModeIsActive()) {
            // teleop logic
            r.telemetryAprilTag();
            // Push telemetry to the Driver Station.
            telemetry.update();

            if (r.buffer) {
                if (r.runtime.milliseconds() - r.prevBuffer >= r.bufferTime) {
                    // reset buffer
                    r.buffer = false;
                    r.prevBuffer = 0;

                    // move the claw down
                    r.passoverServoLeft.setPosition(0);
                    r.passoverServoRight.setPosition(0);
                }
            }

            // buffer logic (for hardstop)
            if (buffer) {
                hardstopBufferAction(); // buffer logic for the hardstop claw
            }

            // stop the vertical linear slides if they are not busy
            if (!r.motorLeftVert.isBusy()) r.motorLeftVert.setPower(0);
            if (!r.motorRightVert.isBusy()) r.motorRightVert.setPower(0);

            double max;
            // don't question it. I don't know why it works like this. It just does.
            double y = -gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
            double x = gamepad1.left_stick_x; //x
            double rx = 0.8 * gamepad1.right_stick_x;//rx
//            double y2 = gamepad2.left_stick_y * .75 ;

            //will reset the direction of the robot relative of the position it is currently at
            if (gamepad1.options) {
                r.imu.resetYaw();
            }

            double heading = r.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            //does math stuff to keep directions constant
            double adjustedX = y * Math.sin(heading) + x * Math.cos(heading);
            double adjustedY = y * Math.cos(heading) - x * Math.sin(heading);

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
            double default_power = .80;
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

            if (gamepad1.left_bumper && !r.toggleSwitch) {
                r.toggleSwitch = true;
            } else if (gamepad1.left_bumper && r.toggleSwitch) {
                r.toggleSwitch = false;
            }

            //converts from field oriented drive to robot oriented drive
//            if (!gamepad1.left_bumper & r.toggleSwitch) {
//                leftFrontPower = y + x + rx;
//                rightFrontPower = y - x - rx;
//                leftBackPower = y - x + rx;
//                rightBackPower = y + x - rx;
//            }

            // drive
            r.frontLeftDrive.setPower(leftFrontPower * power);
            r.frontRightDrive.setPower(rightFrontPower * power);
            r.backLeftDrive.setPower(leftBackPower * power);
            r.backRightDrive.setPower(rightBackPower * power);


            //close intake and open outtake
            if (gamepad2.right_trigger > 0) {
                r.closeIntake();
                r.openOuttake();
            }

//            if (gamepad2.right_trigger > 0) {
//                outtakeClawServo.setPosition(servoAngle(120));
//                intakeClawServo.setPosition(servoAngle(45));
//            }

            //open intake and close outtake
            if (gamepad2.left_trigger > 0) {
                r.openIntake();
                r.closeOuttake();
            }
//
//            if (gamepad2.left_trigger > 0) {
//                outtakeClawServo.setPosition(servoAngle(45));
//                intakeClawServo.setPosition(servoAngle(120));
//            }

            // extend
            if (gamepad2.a) {
                //SUBJECT TO CHANGE DUE TO MECHANICAL AND SERVO RANGE
                //also set servo range to the specific rotation we need
                r.extendHorizontalSlides();
                r.dropDiffIntake();

                // begin hardstop buffer
                startHardstopBuffer();
            }

            // retract
            if (gamepad2.b) {
                r.retractHorizontalSlides();
                // diff intake might be too slow during transition, may need to add buffer
                r.raiseDiffIntake();
            }

            //pivot
            if (gamepad2.y) {
                r.pivotPassover();
            }
            //unpivot
            if (gamepad2.x) {
                r.unpivotPassover();
            }

            // i think we will leave these in for failsafes
            if (gamepad1.a){
                if (r.horizontalSlidesExtended) break; // if slides are extended, do not activate hardstop
                r.activateHardStop();
            }

            if (gamepad1.b){
                r.disableHardStop();
            }

            //grabs specimen from wall
            if (gamepad2.right_bumper) {
                r.specimenOffWall();
            }

            if (gamepad2.dpad_up) {
                //telling the motor to rotate one inch
                r.positionTopOuttake();
                r.outtakeServoTopDropoff();
            }

            if (gamepad2.dpad_right) {
                r.positionMidOuttake();
                r.specimenOffWall();
            }

            if (gamepad2.dpad_down) {
                r.resetOuttakeSlides();
                r.resetOuttakeServo();
            }


            if (gamepad2.dpad_left) {
                r.scoreSpecimen();
            }

            /* This gives us data un parts of the robot we want
            A template to do this is:
            telementry.addDate("What you want the name of the data to show up as ", [mechanism you want to check].getCurrentPosition());
            Now some parts can very depending what you want
            Now for example, you can get the ticks of a motor or continuous servo by using '.getCurrentPosition'
                 */
            telemetry.addData("Status", "Run Time: " + r.runtime.toString());
            telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
            telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);
            telemetry.addData("Motor ticks 1", r.motorLeftVert.getCurrentPosition());
            telemetry.addData("Motor ticks 2", r.motorRightVert.getCurrentPosition());
//            telemetry.addData("Servo position", servoAngle(servoTarget));
            telemetry.update();
        }
        // end teleop mode

        r.visionPortal.close();
        // saves CPU
    }
}
