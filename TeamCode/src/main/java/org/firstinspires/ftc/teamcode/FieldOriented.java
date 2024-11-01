package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.IMU;

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

@TeleOp(name="Field Oriented Drive", group="Linear OpMode")
public class FieldOriented extends LinearOpMode {

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

    //toggleSwitch between Field Centric Drive to Robot Centric Drive
    boolean toggleSwitch = false;

    @Override
    public void runOpMode() {
        frontLeftDrive = hardwareMap.get(DcMotor.class, "motor0");
        backLeftDrive = hardwareMap.get(DcMotor.class, "motor1");
        frontRightDrive = hardwareMap.get(DcMotor.class, "motor2");
        backRightDrive = hardwareMap.get(DcMotor.class, "motor3");
        // ########################################################################################
        // !!!            IMPORTANT Drive Information. Test your motor directions.            !!!!!
        // ########################################################################################
        // Most robots need the motors on one side to be reversed to drive forward.
        // The motor reversals shown here are for a "direct drive" robot (the wheels turn the same direction as the motor shaft)
        // If your robot has additional gear reductions or uses a right-angled drive, it's important to ensure
        // that your motors are turning in the correct direction.  So, start out with the reversals here, BUT
        // when you first test your robot, push the left joystick forward and observe the direction the wheels turn.
        // Reverse the direction (flip FORWARD <-> REVERSE ) of any wheel that runs backward
        // Keep testing until ALL the wheels move the robot forward when you push the left joystick forward.

        //If it does not work properly you may have to remove motor0 and motor1 setDirection.
        //If that doesn't work then switch the forwards and reversed.
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

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            double max;

            // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
            double y   = -gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
            double x =  gamepad1.left_stick_x; //x
            double rx     =  gamepad1.right_stick_x; //rx

//removed negative from here
            if (gamepad1.options){
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
            double leftFrontPower  = adjustedY + adjustedX + rx;
            double rightFrontPower = adjustedY - adjustedX - rx;
            double leftBackPower   = adjustedY - adjustedX + rx;
            double rightBackPower  = adjustedY + adjustedX - rx;

            // Normalize the values so no wheel power exceeds 100%
            // This ensures that the robot maintains the desired motion.
            max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
            max = Math.max(max, Math.abs(leftBackPower));
            max = Math.max(max, Math.abs(rightBackPower));
            //sets default speed
           double power = 0.7;
           //slow mode
           if (gamepad1.right_trigger > 0){
              power =  (power * 0.5);
           }
           //fast mode
            if (gamepad1.left_trigger > 0){
                power = 1;
            }

//            if (max > 1.0) {
//                leftFrontPower /= max;
//                rightFrontPower /= max;
//                leftBackPower /= max;
//                rightBackPower /= max;
//            }

            if(gamepad1.left_bumper && !toggleSwitch) {
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

            frontLeftDrive.setPower(leftFrontPower * power);
            frontRightDrive.setPower(rightFrontPower * power);
            backLeftDrive.setPower(leftBackPower * power);
            backRightDrive.setPower(rightBackPower * power);

            // IMU testing

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
            telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);
            telemetry.update();

        }
    }
}
